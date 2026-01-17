#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
#include "driver/ledc.h"

#define LEFT_M0 13
#define LEFT_M1 12
#define RIGHT_M0 14
#define RIGHT_M1 15
int speed = 150;
int noStop = 0;
long long last = 0;
const int freq = 2000;
const int motorPWMChannnel = 8;
const int lresolution = 8;
volatile unsigned int motor_speed = 100;
void robot_setup();
void robot_fwd(int left, int right);
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 250;
unsigned int get_speed(unsigned int sp) {
  return map(sp, 0, 100, 0, 255);
}

void robot_setup() {
  // Configure LEDC timer - Use TIMER_1 to avoid conflicts with camera
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,  // 8-bit resolution
    .timer_num = LEDC_TIMER_1,            // Use TIMER_1 to avoid conflicts with camera
    .freq_hz = 2000,                      // Frequency 2000Hz
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
  // Configure 4 channels - Use channels 4-7 to avoid conflicts with camera
  ledc_channel_config_t ledc_channel[4] = {
    { .gpio_num = LEFT_M0,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_4,  // Use channel 4
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,  // Use TIMER_1
      .duty = 0,
      .hpoint = 0 },
    { .gpio_num = LEFT_M1,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_5,  // Use channel 5
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,  // Use TIMER_1
      .duty = 0,
      .hpoint = 0 },
    { .gpio_num = RIGHT_M0,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_6,  // Use channel 6
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,  // Use TIMER_1
      .duty = 0,
      .hpoint = 0 },
    { .gpio_num = RIGHT_M1,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_7,  // Use channel 7
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,  // Use TIMER_1
      .duty = 0,
      .hpoint = 0 }
  };

  for (int i = 0; i < 4; i++) {
    ledc_channel_config(&ledc_channel[i]);
  }

  pinMode(33, OUTPUT);
  robot_fwd(150, 150);
}

void robot_stop() {
  robot_fwd(150, 150);
}

void robot_fwd(int right, int left) {
  if (left < 0) {
    left = 0;
  }

  if (right < 0) {
    right = 0;
  }

  if (left > 300) {
    left = 300;
  }

  if (right > 300) {
    right = 300;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, left < 150 ? 150 - left : 0); 
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, left >= 150 ? left - 150 : 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, right < 150 ? 150 - right : 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, right >= 150 ? right - 150 : 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);
}

extern int gpLed;

extern String WiFiAddr;

typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.printf("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.printf("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    // Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
    //      ,(uint32_t)(_jpg_buf_len),
    //      (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
    //      avg_frame_time, 1000.0 / avg_frame_time
    //  );
  }

  last_frame = 0;
  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  String page = "";
  page += "<!DOCTYPE html><html lang='ru'><head><meta charset='UTF-8'><title>Управление</title><meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=no'>";
  page += "<style> body { background: #111; margin: 0; height: 100vh; display: flex; flex-direction: column; align-items: center; color: #0f0; font-family: Arial, sans-serif; } #pad-wrapper { flex: 1;";
  page += "width: 100%; display: flex; justify-content: center; align-items: center; } #pad { background: #222; border: 3px solid #0f0; border-radius: 14px; touch-action: none; position: relative; }";
  page += "#dot { width: 32px; height: 32px; background: red; border-radius: 50%; position: absolute; left: 50%; top: 50%; transform: translate(-50%, -50%); pointer-events: none; }";
  page += ".switch { height: 12vh; min-height: 70px; display: flex; align-items: center; gap: 14px; font-size: 22px; } .switch input { width: 64px; height: 34px; } </style></head><body>";
  page += "<p align=center><IMG SRC='http://" + WiFiAddr + ":81/stream' style='width:300px; transform:rotate(0deg);'></p><br/><div id='pad-wrapper'><div id='pad'><div id='dot'></div></div></div>";
  page += "<div class='switch'><label>Свет</label><input type='checkbox' id='lightSwitch'></div><script>var xhttp = new XMLHttpRequest(); var start = Date.now();</script><script>function getsend(arg) { xhttp.open('GET', arg +'?' + new Date().getTime(), true); xhttp.send() } </script><script>const pad = document.getElementById('pad');const dot = document.getElementById('dot');const lightSwitch = document.getElementById('lightSwitch');";
  page += "const ESP_URL = 'http://" + WiFiAddr + "'; let lastSend = 0; const SEND_INTERVAL = 40; function resizePad() { const switchHeight = document.querySelector('.switch').offsetHeight; ";
  page += "const size = Math.min(window.innerWidth * 0.96, (window.innerHeight - switchHeight) * 0.96); pad.style.width = size + 'px'; pad.style.height = size + 'px'; }";
  page += "window.addEventListener('resize', resizePad); resizePad(); pad.addEventListener('touchstart', handleTouch); pad.addEventListener('touchmove', handleTouch);";
  page += "pad.addEventListener('touchend', resetCenter); pad.addEventListener('touchcancel', resetCenter); function handleTouch(e) { e.preventDefault();  const touch = e.touches[0];  if (!touch) return;";
  page += "const rect = pad.getBoundingClientRect(); let x = touch.clientX - rect.left; let y = touch.clientY - rect.top; x = Math.max(0, Math.min(x, rect.width)); y = Math.max(0, Math.min(y, rect.height));";
  page += "let xPercent = Math.round((x / rect.width) * 200) - 100; let yPercent = Math.round((y / rect.height) * 200) - 100; let angle = -90 + Math.atan2(xPercent, yPercent) * 180 / Math.PI; if (angle < 0) { angle += 360; }";
  page += "let accel = -yPercent * 1.5; let xRes = accel;  let yRes = accel;if(xPercent < -10){ yRes  *= (100 + xPercent)/ 100;}else if(xPercent > 10){xRes  *= (100 - xPercent)/ 100;} xRes += 150; yRes += 150;";
  page += "dot.style.left = x + 'px'; dot.style.top = y + 'px'; throttledSend(Math.round(xRes),Math.round( yRes)); } function resetCenter() { const rect = pad.getBoundingClientRect(); dot.style.left = (rect.width / 2) + 'px';";
  page += "dot.style.top = (rect.height / 2) + 'px'; sendGet(150, 150, Date.now()-start); } lightSwitch.addEventListener('change', () => { sendLight(lightSwitch.checked ? 'on' : 'off'); });";
  page += "function throttledSend(x, y) { const now = Date.now(); if (now - lastSend > SEND_INTERVAL) { sendGet(x, y, now-start); lastSend = now; } } function sendGet(x, y, n) { fetch(`${ESP_URL}/go?left=${x}&right=${y}&now=${n}`).catch(() => {sendGet(x, y, n);});";
  page += "} function sendLight(state) { if(state == 'on'){  fetch(`${ESP_URL}/ledon`).catch(() => {}); }  else{  fetch(`${ESP_URL}/ledoff`).catch(() => {});  }}</script></body></html>";

  return httpd_resp_send(req, &page[0], strlen(&page[0]));
}

static esp_err_t go_handler(httpd_req_t *req) {
  Serial.println("Go");

  char query[128];
  char param[32];
  int left = 150;
  int right = 150;
  long long n;

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    Serial.printf("Query string: %s\n", query);

    if (httpd_query_key_value(query, "left", param, sizeof(param)) == ESP_OK) {
      left = atoi(param);

      Serial.printf("left: %d\n", left);
    }

    // Параметр delay (пример числового параметра)
    if (httpd_query_key_value(query, "right", param, sizeof(param)) == ESP_OK) {
      right = atoi(param);

      Serial.printf("right: %d\n", right);
    }

    // Параметр delay (пример числового параметра)
    if (httpd_query_key_value(query, "now", param, sizeof(param)) == ESP_OK) {
      n = atoll(param);

      Serial.printf("now: %d\n", n);
    }
  }

  if (n > last) {
    last = n;
    robot_fwd(left, right);
  }

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

static esp_err_t ledon_handler(httpd_req_t *req) {
  digitalWrite(gpLed, HIGH);
  Serial.println("LED ON");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}
static esp_err_t ledoff_handler(httpd_req_t *req) {
  digitalWrite(gpLed, LOW);
  Serial.println("LED OFF");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, "OK", 2);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t go_uri = {
    .uri = "/go",
    .method = HTTP_GET,
    .handler = go_handler,
    .user_ctx = NULL
  };

  httpd_uri_t ledon_uri = {
    .uri = "/ledon",
    .method = HTTP_GET,
    .handler = ledon_handler,
    .user_ctx = NULL
  };

  httpd_uri_t ledoff_uri = {
    .uri = "/ledoff",
    .method = HTTP_GET,
    .handler = ledoff_handler,
    .user_ctx = NULL
  };

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  ra_filter_init(&ra_filter, 20);
  Serial.printf("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &go_uri);
    httpd_register_uri_handler(camera_httpd, &ledon_uri);
    httpd_register_uri_handler(camera_httpd, &ledoff_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
