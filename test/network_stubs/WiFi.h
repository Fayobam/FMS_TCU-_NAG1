#pragma once
#include "Arduino.h"
#include <ArduinoJson.h>
#include <vector>
#include <string>
inline bool operator==(const String& a,const char* b){return a.s==b;}
inline size_t strlcpy(char* dest,const char* src,size_t cap){size_t n=strlen(src);if(cap){size_t take=std::min(n,cap-1);memcpy(dest,src,take);dest[take]=0;}return n;}
namespace ArduinoJson {
template<> struct Converter<String> {
    static void toJson(const String& src,JsonVariant dst){dst.set(src.s);}
};
}
enum {WIFI_OFF,WIFI_STA,WIFI_AP,WIFI_AP_STA};
enum {WL_DISCONNECTED,WL_CONNECTED};
constexpr int WIFI_SCAN_RUNNING=-1,WIFI_SCAN_FAILED=-2;
struct IPAddress {
    int a,b,c,d; IPAddress(int a=0,int b=0,int c=0,int d=0):a(a),b(b),c(c),d(d){}
    String toString() const {return String((std::to_string(a)+"."+std::to_string(b)+"."+std::to_string(c)+"."+std::to_string(d)).c_str());}
};
struct WiFiStub {
    struct Visible{std::string ssid;int rssi;};
    int link=WL_DISCONNECTED,modeValue=0,result=WIFI_SCAN_RUNNING,scans=0,attempts=0,apAttempts=0;
    bool apResult=true,ap=false;std::string selected;std::vector<Visible> visible;
    void persistent(bool){}void setAutoReconnect(bool){}void setHostname(const char*){}void setSleep(bool){}
    void mode(int v){modeValue=v;}
    void disconnect(bool,bool){link=WL_DISCONNECTED;}
    void softAPdisconnect(bool){ap=false;}
    bool softAPConfig(IPAddress,IPAddress,IPAddress){return true;}
    bool softAP(const char*,const char*,int,bool,int){++apAttempts;ap=apResult;return apResult;}
    int scanNetworks(bool async){assert(async);++scans;result=WIFI_SCAN_RUNNING;return result;}
    int scanComplete(){return result;}
    void scanDelete(){visible.clear();result=WIFI_SCAN_FAILED;}
    void begin(const char* ssid,const char*){selected=ssid;++attempts;}
    int status() const{return link;}
    String SSID(int i)const{return String(visible[i].ssid.c_str());}
    String SSID()const{return String(selected.c_str());}
    int RSSI(int i)const{return visible[i].rssi;}int RSSI()const{return -50;}
    IPAddress localIP()const{return {192,168,1,50};}IPAddress softAPIP()const{return {192,168,4,1};}
};
inline WiFiStub WiFi;
