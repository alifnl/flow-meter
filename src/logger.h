#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "webserver_setup.h"
#include <time.h>
#include <dirent.h>
#define HEADER_STRING "epoch;Total;Inst;pulse\r\n"
#define HEADER_SIZE (sizeof(HEADER_STRING) - 1)  
#define TIME_OFFSET 25200 //in seconds (7 hour)
extern int32_t param_value[];

uint64_t epoch =0;

long read_epoch_first_entry(const char* path);
//time lapse is in second
void init_logger(const char* csv_header, const char* file_name, int time_lapse);
bool exists(const char* path);
String epoch_to_date(long epoch_input);
void append_file(const char* path, const char* message);
void list_files();
void get_epoch(const char *line, char *epoch) {
    sscanf(line, "%[^;]", epoch);
}

String epoch_to_date(long epoch_input){
    time_t epoch_time = epoch_input;  // Replace with your epoch time
    struct tm timeinfo;
    // Convert epoch time to struct tm
    if (gmtime_r(&epoch_time, &timeinfo) == NULL) {
        //ESP_LOGE("Time", "Failed to convert epoch time");
        return "";
    }
    char buff[11];
    // Format the date in DD:MM:YYYY format
    strftime(buff, 11, "%d-%m-%Y", &timeinfo);
    //Serial.println(String(file_buffer));
    return String(buff);
}

bool exists(const char* path){
    char buff[64];
    snprintf(buff, 64, "/data/%s", path);  // Safely build the full path
    // opening file
    FILE *f = fopen(buff, "r"); // need to specify the partition first then the name file
    if (f == NULL) //no file return -1
    {
        return false;
    }
    fclose(f);
    return true;

}

void delete_file(const char* path){
    char buff[64];
    snprintf(buff, 64, "/data/%s", path);  // Safely build the full path
    if (exists(path)){
        remove(buff);
    }
    
}
void append_file(const char* path, const char* message){ //append message to a file (tested)
    char buff[64];
    snprintf(buff, 64, "/data/%s", path);  // Safely build the full path
	FILE * file = fopen(buff, "a");
    if (file == NULL) //no file return -1
    {
        Serial.println("- failed to open file for appending");
        return;
    }
	fprintf(file, "%s", message);
	fclose(file);
}

//will return oldest files (csv);
void list_files(char* oldest_file) {
    oldest_file[0] = '\0';
    static uint32_t deleted_file_value = 99999999;
    DIR *d = opendir("/data");  // Open root directory
    if (d == NULL) {
        //ESP_LOGE("File Listing", "Failed to open directory");
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        
        if (strstr(entry->d_name, ".csv")) {
            int day, month, year, result;
            sscanf(entry->d_name, "%d-%d-%d", &day, &month, &year);
            // Convert to integer format YYYYMMDD
            result = (year * 10000) + (month * 100) + day;
            if(result < deleted_file_value){
                deleted_file_value = result;
                memcpy(oldest_file, entry->d_name, strlen(entry->d_name));
                oldest_file[strlen(entry->d_name)] = '\0';  // Manually null-terminate
            }
            Serial.printf("Found file: %s - %d\n", entry->d_name, result);
        }
    }
    Serial.println(oldest_file);
    closedir(d);  // Close the directory when done
}

void housekeeping(){
    //ESP_LOGI("housekeep", "STARTED");
    char delete_file_name[64];
    list_files(delete_file_name);
    //Serial.printf("deleting file %s\n", delete_file_name);
    //String file_name_local = epoch_to_date(epoch+TIME_OFFSET- 259200) + ".csv";
    delete_file(delete_file_name);
    //ESP_LOGI("housekeep", "FINISHED");
}

#endif