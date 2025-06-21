#include <MaskerClassification_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// LED Built-in pin untuk XIAO ESP32S3
#define LED_BUILTIN 21

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
#define INFERENCE_INTERVAL_MS 2000
#define CONFIDENCE_THRESHOLD 0.7
#define BLINK_INTERVAL_MS 500  // Interval untuk kelap-kelip

static bool debug_nn = false, is_initialised = false;
static uint8_t *snapshot_buf = nullptr;
static unsigned long last_inference_time = 0;
static unsigned long last_blink_time = 0;
static bool led_blink_state = false;
static int inference_count = 0;
String previous_class = "";
float previous_confidence = 0.0;

// Enum untuk status LED
enum LedStatus {
  LED_OFF,      // TidakAda - LED mati
  LED_ON,       // PakaiMasker - LED menyala
  LED_BLINK     // TidakPakai - LED kelap-kelip
};

static LedStatus current_led_status = LED_OFF;

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,
  .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 15,
  .fb_count = 1, .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void setup() {
  Serial.begin(115200); delay(1000);
  
  // Inisialisasi LED built-in
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // LED mati di awal
  
  Serial.println("🎭 DETEKSI MASKER - ESP32S3 + Edge Impulse");
  Serial.println("💡 LED Control: PakaiMasker=ON, TidakPakai=BLINK, TidakAda=OFF");
  
  if (!ei_camera_init()) { 
    Serial.println("❌ Gagal init kamera!"); 
    while (1) delay(1000); 
  }
  
  Serial.println("✅ Kamera OK\nTunggu 3 detik..."); 
  delay(3000);
  last_inference_time = millis();
  last_blink_time = millis();
}

void loop() {
  // Handle LED blinking
  handle_led_control();
  
  if (millis() - last_inference_time < INFERENCE_INTERVAL_MS) { 
    delay(100); 
    return; 
  }
  
  last_inference_time = millis(); 
  inference_count++;
  
  if (!allocate_snapshot_buffer()) { 
    Serial.println("❌ Gagal alokasi buffer"); 
    delay(1000); 
    return; 
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  Serial.printf("📸 Gambar #%d...\n", inference_count);
  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
    Serial.println("❌ Gagal ambil gambar"); 
    free_snapshot_buffer(); 
    delay(1000); 
    return;
  }

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("❌ Inference gagal (%d)\n", err); 
    free_snapshot_buffer(); 
    delay(1000); 
    return;
  }

  print_result(&result);
  free_snapshot_buffer();
}

void handle_led_control() {
  switch (current_led_status) {
    case LED_OFF:
      digitalWrite(LED_BUILTIN, LOW);
      break;
      
    case LED_ON:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
      
    case LED_BLINK:
      if (millis() - last_blink_time >= BLINK_INTERVAL_MS) {
        led_blink_state = !led_blink_state;
        digitalWrite(LED_BUILTIN, led_blink_state);
        last_blink_time = millis();
      }
      break;
  }
}

void update_led_status(String class_name, float confidence) {
  LedStatus new_status = LED_OFF;  // Default
  
  if (confidence >= CONFIDENCE_THRESHOLD) {
    if (class_name.equalsIgnoreCase("PakaiMasker")) {
      new_status = LED_ON;
    } else if (class_name.equalsIgnoreCase("TidakPakai")) {
      new_status = LED_BLINK;
    } else if (class_name.equalsIgnoreCase("TidakAda")) {
      new_status = LED_OFF;
    }
  }
  
  // Update status LED jika berubah
  if (new_status != current_led_status) {
    current_led_status = new_status;
    
    // Reset blink state saat mengubah mode
    if (current_led_status == LED_BLINK) {
      led_blink_state = false;
      last_blink_time = millis();
    }
    
    // Print status LED
    switch (current_led_status) {
      case LED_OFF:
        Serial.println("💡 LED: OFF (TidakAda)");
        break;
      case LED_ON:
        Serial.println("💡 LED: ON (PakaiMasker)");
        break;
      case LED_BLINK:
        Serial.println("💡 LED: BLINK (TidakPakai)");
        break;
    }
  }
}

void print_result(ei_impulse_result_t *res) {
  Serial.printf("⏱️ DSP=%dms, Klasifikasi=%dms\n", res->timing.dsp, res->timing.classification);
  float max_conf = 0; 
  const char* pred_class = "Unknown";
  
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float conf = res->classification[i].value;
    const char* label = ei_classifier_inferencing_categories[i];
    if (conf > max_conf) { 
      max_conf = conf; 
      pred_class = label; 
    }
    if (conf >= CONFIDENCE_THRESHOLD)
      Serial.printf("   ✅ %s: %.1f%%\n", label, conf * 100);
    else if (conf > 0.1)
      Serial.printf("   ⚪ %s: %.1f%%\n", label, conf * 100);
  }

  String current_class = String(pred_class);
  bool update = false;
  if (max_conf >= CONFIDENCE_THRESHOLD) update = true;
  else if (current_class != previous_class && (max_conf - previous_confidence) > 0.2) update = true;

  if (update) {
    previous_class = current_class;
    previous_confidence = max_conf;
    
    // Update LED status berdasarkan hasil klasifikasi
    update_led_status(current_class, max_conf);
  }

  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  if (previous_confidence >= CONFIDENCE_THRESHOLD) {
    if (previous_class.indexOf("mask") >= 0 && previous_class.indexOf("no") == -1)
      Serial.printf("🟢 MASKER (%.1f%%)\n", previous_confidence * 100);
    else if (previous_class.indexOf("no") >= 0)
      Serial.printf("🔴 TIDAK MASKER (%.1f%%)\n", previous_confidence * 100);
    else
      Serial.printf("🟡 %s (%.1f%%)\n", previous_class.c_str(), previous_confidence * 100);
  } else {
    Serial.printf("🟡 TIDAK YAKIN (%.1f%%)\n", previous_confidence * 100);
  }
}

bool allocate_snapshot_buffer() {
  if (snapshot_buf) free(snapshot_buf);
  snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
  return (snapshot_buf != nullptr);
}

void free_snapshot_buffer() {
  if (snapshot_buf) { 
    free(snapshot_buf); 
    snapshot_buf = nullptr; 
  }
}

bool ei_camera_init() {
  if (is_initialised) return true;
  if (esp_camera_init(&camera_config) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 0); 
  s->set_hmirror(s, 0);
  is_initialised = true; 
  return true;
}

bool ei_camera_capture(uint32_t w, uint32_t h, uint8_t *out_buf) {
  if (!is_initialised) return false;
  camera_fb_t *fb = esp_camera_fb_get(); 
  if (!fb) return false;
  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf); 
  esp_camera_fb_return(fb);
  if (!ok) return false;
  if (w != EI_CAMERA_RAW_FRAME_BUFFER_COLS || h != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)
    ei::image::processing::crop_and_interpolate_rgb888(out_buf, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS, out_buf, w, h);
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pix = offset * 3, i = 0;
  while (length--) { 
    out_ptr[i++] = (snapshot_buf[pix + 2] << 16) + (snapshot_buf[pix + 1] << 8) + snapshot_buf[pix]; 
    pix += 3; 
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
