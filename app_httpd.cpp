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
void movement_watchdog();
void robot_fwd(int left, int right);
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 250;
static int64_t last_ws_cmd_us = 0;
static const int64_t WS_TIMEOUT_US = 300000; // 300 мс
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
  robot_fwd(0, 0);
}

void robot_stop() {
  robot_fwd(0, 0);
}

void robot_fwd(int left, int right) {

  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, left < 0 ? -left : 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, left >= 0 ? left : 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, right < 0 ? -right : 0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, right >= 0 ? right : 0);
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

  String page;
  page.reserve(9000);

  page += "<!DOCTYPE html><html lang='ru'><head><meta charset='UTF-8'>";
  page += "<meta name='viewport' content='width=device-width,height=device-height,initial-scale=1.0,maximum-scale=1.0,user-scalable=no'>";
  page += "<title>ESP32 CAM ROBOT</title>";

  page += "<style>";
  page += "html,body{margin:0;padding:0;width:100%;height:100%;background:#111;overflow:hidden;color:#0f0;font-family:Arial}";
  page += "#main{display:flex;width:100vw;height:100%}";
  page += ".track{width:15vw;height:100%;background:#222;border:2px solid #0f0;position:relative;touch-action:none}";
  page += ".center-line{position:absolute;top:50%;left:0;right:0;height:2px;background:#0f0;opacity:.3}";
  page += ".fill{position:absolute;left:0;right:0}";
  page += ".fill.up{bottom:50%;background:#0f0}";
  page += ".fill.down{top:50%;background:red}";
  page += ".value{position:absolute;top:8px;left:50%;transform:translateX(-50%);font-size:20px}";
  page += "#center{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center}";
  page += "#center img{max-width:100%;max-height:calc(100% - 50px);border:3px solid #0f0}";
  page += "</style></head><body>";

  page += "<div id='main'>";
  page += "<div id='leftTrack' class='track'><div class='value' id='leftVal'>0</div><div class='center-line'></div><div class='fill up' id='leftUp'></div><div class='fill down' id='leftDown'></div></div>";
  page += "<div id='center'><img src='http://" + WiFiAddr + ":81/stream'><label><input type='checkbox' id='lightSwitch'> Свет</label></div>";
  page += "<div id='rightTrack' class='track'><div class='value' id='rightVal'>0</div><div class='center-line'></div><div class='fill up' id='rightUp'></div><div class='fill down' id='rightDown'></div></div>";
  page += "</div>";

  // ====== JS ======
page += "<script>";
page += "const MAX=150;";
page += "const SEND_INTERVAL=50;";
page += "const DEADMAN=300;";

page += "let l=0,r=0,ll=null,rr=null;";
page += "let lastSend=0,lastAct=Date.now();";
page += "let lt=null,rt=null;";
page += "let ws=null;";

page += "function wsConnect(){";
page += "ws=new WebSocket(`ws://${location.host}/ws`);";
page += "ws.onclose=()=>setTimeout(wsConnect,500);";
page += "}";
page += "wsConnect();";

page += "function calc(y,h){";
page += "let p=1-y/h;";
page += "let v=Math.round(p*MAX*2-MAX);";
page += "return Math.max(-MAX,Math.min(MAX,v));";
page += "}";

page += "function ui(s,v,h){";
page += "let u=document.getElementById(s+'Up');";
page += "let d=document.getElementById(s+'Down');";
page += "document.getElementById(s+'Val').textContent=v;";
page += "u.style.height=d.style.height='0px';";
page += "let px=Math.abs(v)/MAX*(h/2);";
page += "if(v>0)u.style.height=px+'px';";
page += "if(v<0)d.style.height=px+'px';";
page += "}";

page += "function sendWS(force=false){";
page += "if(!ws||ws.readyState!==1)return;";
page += "let n=Date.now();";
page += "if(!force&&n-lastSend<SEND_INTERVAL)return;";
page += "if(!force&&l===ll&&r===rr)return;";
page += "lastSend=n;ll=l;rr=r;lastAct=n;";
page += "ws.send(l+','+r);";
page += "}";

page += "function stopNow(){";
page += "l=0;r=0;ll=rr=null;";
page += "sendWS(true);";
page += "}";

page += "setInterval(()=>{";
page += "if(Date.now()-lastAct>DEADMAN){";
page += "if(l||r)stopNow();";
page += "}},100);";

page += "function bind(el,side){";

page += "el.addEventListener('touchstart',e=>{";
page += "for(let t of e.changedTouches){";
page += "if(side==='left'&&lt===null)lt=t.identifier;";
page += "if(side==='right'&&rt===null)rt=t.identifier;";
page += "}});";

page += "el.addEventListener('touchmove',e=>{";
page += "let rct=el.getBoundingClientRect();";
page += "for(let t of e.changedTouches){";
page += "if(t.identifier===lt&&side==='left'){";
page += "l=calc(t.clientY-rct.top,rct.height);";
page += "ui('left',l,rct.height);}";
page += "if(t.identifier===rt&&side==='right'){";
page += "r=calc(t.clientY-rct.top,rct.height);";
page += "ui('right',r,rct.height);}";
page += "}});";

page += "el.addEventListener('touchend',e=>{";
page += "let stop=false;";
page += "for(let t of e.changedTouches){";
page += "if(t.identifier===lt){lt=null;l=0;stop=true;}";
page += "if(t.identifier===rt){rt=null;r=0;stop=true;}";
page += "}";
page += "if(stop)stopNow();";
page += "});";

page += "el.addEventListener('touchcancel',stopNow);";
page += "}";

page += "bind(leftTrack,'left');";
page += "bind(rightTrack,'right');";

page += "function loop(){";
page += "sendWS();";
page += "requestAnimationFrame(loop);";
page += "}";
page += "requestAnimationFrame(loop);";

page += "window.addEventListener('blur',stopNow);";
page += "document.addEventListener('visibilitychange',()=>{";
page += "if(document.hidden)stopNow();";
page += "});";

page += "lightSwitch.onchange=e=>{";
page += "fetch(e.target.checked?'/ledon':'/ledoff').catch(()=>{});";
page += "};";

page += "</script>";


  page += "</body></html>";

  return httpd_resp_send(req, page.c_str(), page.length());
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
}void movement_watchdog() {
  if (esp_timer_get_time() - last_ws_cmd_us > WS_TIMEOUT_US) {
    robot_stop();
  }
}
static esp_err_t ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    return ESP_OK; // handshake
  }

  httpd_ws_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
  if (ret != ESP_OK) return ret;

  char buf[32];
  frame.payload = (uint8_t*)buf;
  ret = httpd_ws_recv_frame(req, &frame, frame.len);
  if (ret != ESP_OK) return ret;

  buf[frame.len] = 0;

  int left = 0, right = 0;
  if (sscanf(buf, "%d,%d", &left, &right) == 2) {
    last_ws_cmd_us = esp_timer_get_time();
    robot_fwd(left, right);
  }

  return ESP_OK;
}
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

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

  httpd_uri_t ws_uri = {
  .uri = "/ws",
  .method = HTTP_GET,
  .handler = ws_handler,
  .user_ctx = NULL,
  .is_websocket = true
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
    httpd_register_uri_handler(camera_httpd, &ledon_uri);
    httpd_register_uri_handler(camera_httpd, &ledoff_uri);
    httpd_register_uri_handler(camera_httpd, &ws_uri);  
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
