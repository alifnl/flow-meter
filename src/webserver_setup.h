#ifndef WEBSERVER_SETUP_H
#define WEBSERVER_SETUP_H
#include <Arduino.h>
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "logger.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "nvs.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "global_config.h"

#define RESPONSE_BUFFER_SIZE 1024
#define OTA_BUFF_SIZE 256
#define MIN(a, b) ((a) < (b) ? (a) : (b))
extern int32_t param_value[];
extern bool warning_global;
extern const char* param_key_nvs[];
extern nvs_handle_t my_handle_nvs;
extern int32_t meas_vol_shdw;
extern const char* shdw_nvs_key;
extern void update_param_display(int pos, bool logic, bool blink_pointer);
extern int8_t menu_position;
char response_buffer[RESPONSE_BUFFER_SIZE];
char query[32];

static const esp_partition_t *update_partition = NULL;
static esp_ota_handle_t ota_handle = 0;
void spiffs_setup();
httpd_handle_t setup_server();
esp_err_t get_file_handler(httpd_req_t *req);
const char *get_content_type(const char *filename);
esp_err_t ota_post_handler( httpd_req_t *req );

void NTP_config() {
  const char* ntpServer = "pool.ntp.org";
  //init and get the time
  configTime(7*3600, 0, ntpServer, "time.google.com");
}


void spiffs_setup()
{
    // initiate config variable
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/data",
        .partition_label = NULL, // if you have more than 1 partition
        .max_files = 300,
        .format_if_mount_failed = true // be careful not to lose important information
    };

    // register the configuration variable as spiffs
    esp_err_t err_result = esp_vfs_spiffs_register(&config);
    #ifdef DEBUG
    if (err_result != ESP_OK)
    {
        Serial.printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(err_result));
        return;
    }
    #endif
    // declare variable to know the total memory of the flash and used space
    size_t total = 0, used = 0;
    err_result = esp_spiffs_info(config.partition_label, &total, &used);
    #ifdef DEBUG
    if (err_result != ESP_OK)
    {
        Serial.printf("Failed to get partiton info (%s)\n", esp_err_to_name(err_result));
    }
    else
    {
        Serial.printf("Partition size total; %d, used: %d\n", total, used);
    }
    #endif

}
void nvs_setup(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      #ifdef DEBUG
      Serial.println("Erasing NVS due to no free pages or version mismatch...");
      #endif
      if (nvs_flash_erase() == ESP_OK) {
          err = nvs_flash_init();
          #ifdef DEBUG
          Serial.printf("NVS Reinit Result: %s\n", esp_err_to_name(err));
          #endif
      } else {
          #ifdef DEBUG
          Serial.println("NVS Erase Failed!");
          #endif
          return;
      }
    }
    // Check if we should proceed with NVS
    #ifdef DEBUG
    if (err != ESP_OK) {
        Serial.printf("NVS Init Error: %s\n", esp_err_to_name(err));
        return;
    }
    #endif
  
    err = nvs_open("storage", NVS_READWRITE, &my_handle_nvs);
    #ifdef DEBUG
    Serial.printf("NVS Open Result: %s\n", esp_err_to_name(err));
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    #endif
    //get nvs value
    for (int i = MENU_PAGE-1; i >= 0; i-- ){
      nvs_get_i32(my_handle_nvs, param_key_nvs[i], &param_value[i]);
      update_param_display(i,false, false);
    }
    nvs_get_i32(my_handle_nvs, shdw_nvs_key, &meas_vol_shdw);
}

esp_err_t device_get_vol(httpd_req_t *req){
    //ESP_LOGE("HTTP", "start get vol");
    memset(query, 0, sizeof(query));
    int vol_percent = 100*param_value[0]/param_value[5];
    sprintf(query, (param_value[0] * 0.001 >= 100) ? "%.1fL|%.1f%%|%d" 
                              : (param_value[0] *0.001 >= 10) ?  "%.2fL|%.1f%%|%d" : "%.3fL|%.1f%%|%d", 
    param_value[0] / 1000.0, vol_percent / 1000.0, warning_global);  
    httpd_resp_set_status(req, "200 OK");
    //httpd_resp_set_hdr(req, "Connection", "Keep-Alive");
    httpd_resp_send(req, query, strlen(query));
    httpd_req_recv(req, NULL, 0);
    //httpd_resp_send_chunk(req, query, strlen(query));
    //httpd_resp_send_chunk(req, NULL, 0);
    //ESP_LOGE("HTTP", "end get vol");
    return ESP_OK;
}
esp_err_t device_reset_vol(httpd_req_t *req){
    param_value[0] = 0;
    param_value[1] = 0;
    meas_vol_shdw = 0;
    nvs_set_i32(my_handle_nvs, param_key_nvs[0], param_value[0]);
    nvs_set_i32(my_handle_nvs, param_key_nvs[1], param_value[1]);
    nvs_set_i32(my_handle_nvs, shdw_nvs_key, meas_vol_shdw);
    nvs_commit(my_handle_nvs);
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}
esp_err_t device_get_time(httpd_req_t *req){
    //ESP_LOGE("HTTP", "start get time");
    memset(query, 0, sizeof(query));
    snprintf(query, sizeof(query), "%" PRIu64, epoch);
    httpd_resp_set_status(req, "200 OK");
    //httpd_resp_set_hdr(req, "Connection", "Keep-Alive");
    httpd_resp_send(req, query, strlen(query)); 
    return ESP_OK;
}

esp_err_t device_reboot_handler(httpd_req_t *req){
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, "OK", 2);
    vTaskDelay( 1000 / portTICK_RATE_MS);
    esp_restart();
}

esp_err_t device_push_time(httpd_req_t *req){
    // Extract the query string from the URI
    memset(query, 0, sizeof(query));
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {

        // Check if it's a valid numeric input
        bool isNumeric = true;
        for (int i = 0; query[i] != '\0'; i++) {
            if (query[i] < '0' || query[i] > '9') {
                isNumeric = false;
                break;
            }
        }
        epoch = atoi(query);
    }
    // Send response back
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}
esp_err_t device_set_param(httpd_req_t *req){    
    int received = 0;
    int remaining = req->content_len;
    memset(response_buffer, 0, sizeof(response_buffer));
    // Read the POST data in chunks
    while (remaining > 0)
    {
        // Read a chunk of the request body
        int ret = httpd_req_recv(req, response_buffer + received, MIN(remaining, sizeof(response_buffer) - 1 - received));
        if (ret <= 0)
        {
            // Check if it’s an error or end of stream
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                continue; // Retry in case of timeout
            }
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Receiving error");
            return ESP_FAIL; // Exit on other errors
        }

        received += ret;
        remaining -= ret;
    }
    response_buffer[received] = '\0';
    // Parse the POST data as JSON
    cJSON *json = cJSON_Parse(response_buffer);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    bool set_param = false;
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    if (cJSON_IsString(mode))
    {
        if(memcmp(mode->valuestring, "SET", 3)==0) set_param = true;
    }

    cJSON *HF_gain = cJSON_GetObjectItem(json, "HF_gain");
    cJSON *LF_gain = cJSON_GetObjectItem(json, "LF_gain");
    cJSON *thre = cJSON_GetObjectItem(json, "thre");
    cJSON *max_vol = cJSON_GetObjectItem(json, "max_vol");
    if (cJSON_IsNumber(HF_gain) && set_param)
    {
        if (HF_gain->valueint != param_value[2]){
            nvs_set_i32(my_handle_nvs, param_key_nvs[2], HF_gain->valueint);
        }
        param_value[2] = HF_gain->valueint;
    }else{
        cJSON_SetIntValue(HF_gain, param_value[2]);
    }

    if(cJSON_IsNumber(LF_gain) && set_param){
        if (LF_gain->valueint != param_value[3]){
            nvs_set_i32(my_handle_nvs, param_key_nvs[3], LF_gain->valueint);
        }
        param_value[3] = LF_gain->valueint;
    }else{
        cJSON_SetIntValue(LF_gain, param_value[3]);
    }

    if(cJSON_IsNumber(thre) && set_param){
        if (thre->valueint != param_value[4]){
            nvs_set_i32(my_handle_nvs, param_key_nvs[4], thre->valueint);
        }
        param_value[4] = thre->valueint;
    }else{
        cJSON_SetIntValue(thre, param_value[4]);
    }

    if(cJSON_IsNumber(max_vol) && set_param){
        if (max_vol->valueint != param_value[5]){
            nvs_set_i32(my_handle_nvs, param_key_nvs[5], max_vol->valueint);
        }
        param_value[5] = max_vol->valueint;
    }else{
        cJSON_SetIntValue(max_vol, param_value[5]);
    }

    if(set_param){
        httpd_resp_send(req, "OK",2);
        update_param_display(2, menu_position == 2, false);
        update_param_display(3, menu_position == 3, false);
        update_param_display(4, menu_position == 4, false);
        update_param_display(5, menu_position == 5, false);

    }else{
        char *json_str = cJSON_PrintUnformatted(json);
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        free(json_str);
    }
    cJSON_Delete(json);
    return ESP_OK;
}
esp_err_t get_file_handler(httpd_req_t *req)
{
    //ESP_LOGE("HTTP","start serving %s", req->uri);
    char buff[64];
    snprintf(buff, sizeof(buff), "/data%s", (strcmp(req->uri, "/") == 0) ? "/logger.html.gz" : req->uri);

    // opening file
    FILE *file = fopen(buff, "r");
    //File file = SPIFFS.open(line);
  
    // ESP_LOGI (TAG, "opening filepath: %s", line);
    //Serial.println("opening path: " + String(response_buffer));
    if (file == NULL)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }
    memset(response_buffer, 0, sizeof(response_buffer));
    // get_content_type will return the file format type
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, get_content_type(buff));
    if (strstr(buff, ".gz"))
    { // check if the format is gz or not
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=31536000, public, immutable");
    }/*else if (strstr(buff, ".js"))
    { // check if the format is gz or not
        httpd_resp_set_hdr(req, "Cache-Control", "max-age=31536000, public, immutable");
    }*/
    
    size_t bytes_read;
    // read the file as long as the line size
    //ESP_LOGE("HTTP", "begin send chunk");
    while ((bytes_read = fread(response_buffer, 1, sizeof(response_buffer), file)) > 0)
    {
        esp_err_t res = httpd_resp_send_chunk(req, response_buffer, bytes_read);
        if (res != ESP_OK)
        {
            fclose(file);
            return res;
        }
    }

    // don't forget to close the file
    fclose(file);
    //ESP_LOGE("HTTP", "end of send chunk");
    // End the response
    httpd_resp_send_chunk(req, NULL, 0);
    //ESP_LOGE("HTTP","end serving %s", req->uri);
    int sock_fd = httpd_req_to_sockfd(req);
    if (sock_fd >= 0) {
        httpd_sess_trigger_close(req->handle, sock_fd);
    }
    return ESP_OK;
}

esp_err_t serve_csv_handler(httpd_req_t *req) {
    // Extract the filename from the query string
    char file_name[64];
    memset(response_buffer, 0, sizeof(response_buffer));
    size_t filename_len = httpd_req_get_url_query_len(req) + 1;
    if (filename_len > sizeof(file_name)) {
        //ESP_LOGE("FileServer", "Filename is too long");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Filename is too long");
        return ESP_OK;
    }
    // Get the filename from the query string (e.g., ?filename=filename.csv)
    /*Serial.println(String(file_name));
    if(memcmp("reset", file_name, filename_len)==0){
        Serial.println("resetting file");
        return ESP_OK;
    }*/
    if (httpd_req_get_url_query_str(req, file_name, filename_len) == ESP_OK) {
        char* filename_start = strstr(file_name, "");
        if (filename_start) {
            filename_start += strlen(""); // Move past "filename="
            //ESP_LOGI("FileServer", "Requested filename: %s", filename_start);
            // Build the full path to the file in SPIFFS
            char filepath[64];
            snprintf(filepath, sizeof(filepath), "/data/%s", filename_start);

            // Open the file from SPIFFS
            FILE *file = fopen(filepath, "r");
            if (!file) {
                //ESP_LOGE("FileServer", "File not found: %s", filepath);
                httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
                return ESP_OK;
            }
            httpd_resp_set_status(req, "200 OK");
            httpd_resp_set_type(req,"text/plain");
            // Send the file contents to the client
            size_t bytes_read;
            // read the file as long as the line size
            while ((bytes_read = fread(response_buffer, 1, sizeof(response_buffer), file)) > 0)
            {
                esp_err_t res = httpd_resp_send_chunk(req, response_buffer, bytes_read);
                if (res != ESP_OK)
                {
                    fclose(file);
                    return res;
                }
            }
            fclose(file);
            //Serial.println("finish response file @ " + String (filepath));
            // End the response
            httpd_resp_send_chunk(req, NULL,0);
        } else {
            //ESP_LOGE("FileServer", "Query parameter 'filename' not found");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "file not found");
        }
    } else {
        //ESP_LOGE("FileServer", "Failed to get URL query string");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "URL query string failed");
    }
    int sock_fd = httpd_req_to_sockfd(req);
    if (sock_fd >= 0) {
        httpd_sess_trigger_close(req->handle, sock_fd);
    }
    return ESP_OK;
}

httpd_handle_t setup_server(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8196;
    config.max_open_sockets = 10;
    config.backlog_conn = 10;
    config.max_uri_handlers = 9;
    config.send_wait_timeout = 2;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t uri_root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = get_file_handler, // uri handler function
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_root);

        httpd_uri_t csv_handler = {
            .uri = "/csv",
            .method = HTTP_GET,
            .handler = serve_csv_handler, // uri handler function
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &csv_handler);
        //handler push time from browser
        httpd_uri_t push_time = {
            .uri = "/pushTime",
            .method = HTTP_GET,
            .handler = device_push_time, // uri handler function
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &push_time);

        //handler get time from device
        httpd_uri_t get_time = {
            .uri = "/getTime",
            .method = HTTP_GET,
            .handler = device_get_time, // uri handler function
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &get_time);

        httpd_uri_t uri_get_vol = {
            .uri      = "/getVol",
            .method   = HTTP_GET,
            .handler  = device_get_vol,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_get_vol);

        httpd_uri_t uri_reset_vol = {
            .uri      = "/resetVol",
            .method   = HTTP_GET,
            .handler  = device_reset_vol,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_reset_vol);
        
        httpd_uri_t uri_conf_param = {
            .uri      = "/confParam",
            .method   = HTTP_POST,
            .handler  = device_set_param,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_conf_param);

        httpd_uri_t uri_ota_update = {
            .uri      = "/doUpload",
            .method   = HTTP_POST,
            .handler  = ota_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_ota_update); 

        httpd_uri_t uri_reboot = {
            .uri      = "/reboot",
            .method   = HTTP_GET,
            .handler  = device_reboot_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_reboot); 
  }
  return server;
}
const char *get_content_type(const char *filename)
{
    // Find the first occurrence of '.' in the filename
    char *ext = strchr(filename, '.');
    if (!ext)
    {
        return "application/octet-stream"; // Default Content-Type for unknown files
    }
    // Match the extension with a Content-Type
    if (strstr(ext, ".html"))
    {
        return "text/html";
    }
    else if (strstr(ext, ".css"))
    {
        return "text/css";
    }
    else if (strstr(ext, ".js"))
    {
        return "application/javascript";
    }
    else if (strstr(ext, ".json"))
    {
        return "application/json";
    }
    else if (strstr(ext, ".png"))
    {
        return "image/png";
    }
    else if (strstr(ext, ".jpg") || strstr(ext, ".jpeg"))
    {
        return "image/jpeg";
    }
    else if (strstr(ext, ".gif"))
    {
        return "image/gif";
    }
    else if (strstr(ext, ".svg"))
    {
        return "image/svg+xml";
    }
    else if (strstr(ext, ".ico"))
    {
        return "image/x-icon";
    }
    else if (strstr(ext, ".txt"))
    {
        return "text/plain";
    }
    // Default Content-Type
    return "application/octet-stream";
}

esp_err_t ota_post_handler( httpd_req_t *req )
{
    char buf[256];
    char filename[32];
    httpd_resp_set_status( req, HTTPD_500 );    // Assume failure
    int ret, remaining = req->content_len;
    printf( "Receiving\n" );
    httpd_req_get_url_query_str(req, filename, sizeof(filename));
    printf("Filename: %s\n", filename);
    bool is_spiffs_update = false;
    if (strstr(filename, "spiffs.bin") != NULL) {
        is_spiffs_update = true;
        printf("spiffs true\n");

    }
    esp_err_t err = ESP_OK;
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition;
    if (is_spiffs_update) {
        // Find the SPIFFS partition
        update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
        printf("spiffs ota mode\n");
    } else {
        // Find the OTA partition
        update_partition = esp_ota_get_next_update_partition(NULL);
        printf("FW ota mode\n");
    }
    const esp_partition_t *running          = esp_ota_get_running_partition();
  
    if ( update_partition == NULL )
    {
        printf( "Uh oh, bad things\n" );
        goto return_failure;
    }
    printf( "Writing partition: type %d, subtype %d, offset 0x%08x\n", update_partition-> type, update_partition->subtype, update_partition->address);
    printf( "Running partition: type %d, subtype %d, offset 0x%08x\n", running->type,           running->subtype,          running->address);
    
    if(is_spiffs_update){
         // Erase the SPIFFS partition before writing
         err = esp_partition_erase_range(update_partition, 0, update_partition->size);
         if (err != ESP_OK) {
             printf("Failed to erase SPIFFS partition!\n");
             goto return_failure;
         }
    }else{
        err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
        if (err != ESP_OK)
        { 
            printf( "esp_ota_begin failed (%s)", esp_err_to_name(err));
            goto return_failure;
        }
    }
    
    
    if (req->content_len > update_partition->size) {
        printf("SPIFFS update size (%d) exceeds partition size (%d)\n", req->content_len, update_partition->size);
        goto return_failure;
    }

    while ( remaining > 0 )
    {
        // Read the data for the request
        if ( ( ret = httpd_req_recv( req, buf, MIN( remaining, sizeof( buf ) ) ) ) <= 0 )
        {
        if ( ret == HTTPD_SOCK_ERR_TIMEOUT )
        {
            // Retry receiving if timeout occurred
            continue;
        }

        goto return_failure;
        }
        
        size_t bytes_read = ret;
        remaining -= bytes_read;
        if (is_spiffs_update) {
            // Write directly to SPIFFS partition
            /*err = esp_partition_write(update_partition, update_partition->address + (req->content_len - remaining - bytes_read), buf, bytes_read);
            if (err != ESP_OK) {
                printf("Failed to write SPIFFS partition: %s\n", esp_err_to_name(err));
                goto return_failure;
            }*/
            err = esp_partition_write(update_partition, req->content_len - remaining - bytes_read, buf, bytes_read);
            if (err != ESP_OK) {
                printf("Failed to write SPIFFS partition: %s\n", esp_err_to_name(err));
                goto return_failure;
            }
        }else{
            err = esp_ota_write( update_handle, buf, bytes_read);
            if (err != ESP_OK)  goto return_failure;
        }
        
        
    }

    printf( "Receiving done\n" );

    // End response
    if ( ( esp_ota_end(update_handle)                   == ESP_OK ) && 
        ( esp_ota_set_boot_partition(update_partition) == ESP_OK ) )
    {
        printf( "OTA Success?!\n Rebooting\n" );
        fflush( stdout );

        httpd_resp_set_status( req, HTTPD_200 );
        httpd_resp_send( req, NULL, 0 );
        
        vTaskDelay( 2000 / portTICK_RATE_MS);
        esp_restart();
        
        return ESP_OK;
    }
    if(is_spiffs_update){
        printf("SPIFFS OTA SUCESS\n");
        httpd_resp_set_status( req, HTTPD_200 );
        httpd_resp_send( req, NULL, 0 );
        vTaskDelay( 2000 / portTICK_RATE_MS);
        esp_restart();
        //esp_vfs_spiffs_unregister(NULL);  // Unmount SPIFFS
        //spiffs_setup();
        return ESP_OK;
    }
    printf( "OTA End failed (%s)!\n", esp_err_to_name(err));

    return_failure:
        if ( update_handle )
        {
            esp_ota_abort(update_handle);
        }
        
        httpd_resp_set_status( req, HTTPD_500 );    // Assume failure
        httpd_resp_send( req, NULL, 0 );
        return ESP_FAIL;
}

#endif