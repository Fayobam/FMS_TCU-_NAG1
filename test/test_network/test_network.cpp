#include <cassert>
#include <iostream>
#include "NetworkManager.h"

void configure(tcu::NetworkManager& manager,const char* json,bool valid=true){JsonDocument d;assert(!deserializeJson(d,json));assert((manager.configure(d)==nullptr)==valid);}
int main(){
    using Manager=tcu::NetworkManager;
    g_now_ms=1000;Manager manager;manager.begin();assert(manager.apActive());assert(WiFi.modeValue==WIFI_AP_STA);
    for(int i=0;i<10000;++i)manager.update();assert(WiFi.scans==0);
    configure(manager,R"({"op":"mode","mode":1})",false);
    configure(manager,R"({"op":"add","ssid":"Workshop","password":"123"})",false);
    configure(manager,R"({"op":"add","ssid":"Workshop","password":"password1"})");
    configure(manager,R"({"op":"add","ssid":"Home","password":"password2"})");
    g_now_ms+=1001;manager.update();assert(WiFi.scans==1);
    WiFi.visible={{"Home",-30},{"Workshop",-80}};WiFi.result=2;manager.update();assert(WiFi.selected=="Workshop");
    // Bad password / failed association must try the next known visible network.
    g_now_ms+=12000;manager.update();assert(WiFi.selected=="Home");
    WiFi.link=WL_CONNECTED;manager.update();JsonDocument status;manager.describe(status);assert(status["state"]=="CONNECTED");assert(manager.apActive());
    std::string wire;serializeJson(status,wire);assert(wire.find("password1")==std::string::npos);assert(wire.find("password2")==std::string::npos);
    int scans=WiFi.scans;g_now_ms+=120000;manager.update();assert(WiFi.scans==scans);
    // Loss starts recovery without touching the fallback AP.
    WiFi.link=WL_DISCONNECTED;manager.update();assert(WiFi.scans==scans+1);assert(manager.apActive());
    WiFi.visible={{"Unknown",-20}};WiFi.result=1;manager.update();scans=WiFi.scans;
    for(int i=0;i<10000;++i)manager.update();assert(WiFi.scans==scans);
    g_now_ms+=60000;manager.update();assert(WiFi.scans==scans+1);
    configure(manager,R"({"op":"mode","mode":1})");g_now_ms+=1001;manager.update();assert(!manager.apActive());
    // Automatic timeout starts AP even if a station attempt has not succeeded.
    configure(manager,R"({"op":"mode","mode":0})");g_now_ms+=1001;manager.update();assert(!manager.apActive());
    g_now_ms+=20001;manager.update();assert(manager.apActive());
    configure(manager,R"({"op":"mode","mode":2})");g_now_ms+=1001;manager.update();scans=WiFi.scans;
    g_now_ms+=120000;manager.update();assert(WiFi.scans==scans);assert(WiFi.modeValue==WIFI_AP);
    // Failed AP startup retries at a bounded interval, including around millis wrap.
    WiFi={};WiFi.apResult=false;Manager failed;g_now_ms=0xfffff000;failed.begin();int tries=WiFi.apAttempts;
    for(int i=0;i<1000;++i)failed.update();assert(WiFi.apAttempts==tries);
    g_now_ms+=10001;failed.update();assert(WiFi.apAttempts==tries+1);WiFi.apResult=true;g_now_ms+=10001;failed.update();assert(failed.apActive());
    std::cout<<"Network state machine: no networks, priority, timeout, AP fallback, reconnect, scan backoff, modes, AP retry and wrap: PASS\n";
}
