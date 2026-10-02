#include "CommandValidation.h"
#include <cmath>
#include <cstring>

namespace {
struct Rule { const char* key; float min, max; uint8_t count; bool integer; };
const Rule rules[] = {
    {"torque",0,1200,64,true}, {"overrev",3000,9000,0,true}, {"lug",500,2500,0,true},
    {"engPpr",1,240,0,true}, {"outPpr",1,240,0,true}, {"clEn",0,1,0,true},
    {"clKp",0,1000,0,true}, {"kmhRpm",0.001f,1,0,false}, {"transVariant",0,1,0,true},
    {"tcStall",100,350,0,true}, {"tcCoupSr",50,100,0,true}, {"clPwr",0,1,0,true},
    {"clSpeed",0,1,0,true}, {"pFull",4000,25000,0,true},
    {"coefStat",50,255,0,true}, {"coefRel",50,255,0,true},
    {"coefCold",50,255,0,true}, {"coefHot",50,255,0,true},
    {"applyFric",500,12000,4,true}, {"relFric",500,12000,4,true},
    {"applySpring",0,5000,4,true}, {"relSpring",0,5000,4,true},
    {"tmax",100,1200,0,true}, {"tpsC",0,3.3f,0,false}, {"tpsW",0,3.3f,0,false},
    {"map0",-100,260,0,false}, {"mapV",1,300,0,false},
    {"fillp",0,100,4,true}, {"fillt",0,400,4,true}
};
bool number(JsonVariantConst v, float lo, float hi, bool integer = true) {
    if (integer ? !v.is<int>() : !v.is<float>()) return false;
    float f = v.as<float>();
    return std::isfinite(f) && f >= lo && f <= hi;
}
const char* profile(JsonDocument& doc, ControlCommand& out) {
    for (JsonPair kv : doc.as<JsonObject>()) {
        if (!strcmp(kv.key().c_str(), "cmd") || !strcmp(kv.key().c_str(), "requestId")) continue;
        const Rule* rule = nullptr;
        for (const auto& r : rules) if (!strcmp(r.key, kv.key().c_str())) { rule = &r; break; }
        if (!rule) return "Unknown profile field";
        if (rule->count) {
            if (!kv.value().is<JsonArray>() || kv.value().size() != rule->count) return "Invalid profile array length";
            for (JsonVariantConst v : kv.value().as<JsonArrayConst>())
                if (!number(v, rule->min, rule->max, rule->integer)) return "Profile array value out of range";
        } else if (!number(kv.value(), rule->min, rule->max, rule->integer)) return "Profile value out of range";
    }
    out.action = ControlAction::Profile;
    out.profile = *engineProfile.raw();
    EngineProfileData* p = &out.profile;
        JsonArray tq = doc["torque"].as<JsonArray>();
        if ((int)tq.size() >= EP_RPM_BINS * EP_MAP_BINS) {
            for (int i = 0; i < EP_RPM_BINS; i++)
                for (int j = 0; j < EP_MAP_BINS; j++)
                    p->torque[i][j] = (int16_t)constrain(tq[i*EP_MAP_BINS+j].as<int>(), 0, 1200);
        }
        if (doc["overrev"].is<int>()) p->overrev_rpm = (uint16_t)constrain(doc["overrev"].as<int>(), 3000, 9000);
        if (doc["lug"].is<int>())     p->lug_rpm     = (uint16_t)constrain(doc["lug"].as<int>(), 500, 2500);
        if (doc["engPpr"].is<int>())  p->eng_ppr     = (uint16_t)constrain(doc["engPpr"].as<int>(), 1, 240);
        if (doc["outPpr"].is<int>())  p->out_ppr     = (uint16_t)constrain(doc["outPpr"].as<int>(), 1, 240);
        if (doc["clEn"].is<int>())    p->cl_spc_enable = (uint8_t)(doc["clEn"].as<int>() ? 1 : 0);
        if (doc["clKp"].is<int>())    p->cl_spc_kp   = (uint16_t)constrain(doc["clKp"].as<int>(), 0, 1000);
        if (doc["kmhRpm"].is<float>()) p->kmh_per_outrpm = constrain(doc["kmhRpm"].as<float>(), 0.001f, 1.0f);
        if (doc["transVariant"].is<int>()) {
            uint8_t newv = (uint8_t)constrain(doc["transVariant"].as<int>(), 0, (int)TRANS_VARIANT_COUNT - 1);
            if (newv != p->trans_variant) { p->trans_variant = newv; EngineProfile seed; *seed.raw() = *p; seed.seedClutchModelForVariant(newv); *p = *seed.raw(); }
        }
        if (doc["tcStall"].is<int>())   p->tc_stall_mult_x100  = (uint16_t)constrain(doc["tcStall"].as<int>(), 100, 350);
        if (doc["tcCoupSr"].is<int>())  p->tc_coupling_sr_x100 = (uint16_t)constrain(doc["tcCoupSr"].as<int>(), 50, 100);
        if (doc["clPwr"].is<int>())     p->cl_pressure_enable  = (uint8_t)(doc["clPwr"].as<int>() ? 1 : 0);
        if (doc["clSpeed"].is<int>())   p->cl_speed_transitions = (uint8_t)(doc["clSpeed"].as<int>() ? 1 : 0);
        if (doc["pFull"].is<int>())     p->p_full_scale_mbar   = (uint16_t)constrain(doc["pFull"].as<int>(), 4000, 25000);
        if (doc["coefStat"].is<int>())  p->coef_stationary = (uint8_t)constrain(doc["coefStat"].as<int>(), 50, 255);
        if (doc["coefRel"].is<int>())   p->coef_releasing  = (uint8_t)constrain(doc["coefRel"].as<int>(), 50, 255);
        if (doc["coefCold"].is<int>())  p->coef_apply_cold = (uint8_t)constrain(doc["coefCold"].as<int>(), 50, 255);
        if (doc["coefHot"].is<int>())   p->coef_apply_hot  = (uint8_t)constrain(doc["coefHot"].as<int>(), 50, 255);
        { JsonArray af = doc["applyFric"].as<JsonArray>(); JsonArray rf = doc["relFric"].as<JsonArray>();
          JsonArray as = doc["applySpring"].as<JsonArray>(); JsonArray rs = doc["relSpring"].as<JsonArray>();
          if ((int)af.size() >= 4) for (int i=0;i<4;i++) p->apply_friction[i]   = (uint16_t)constrain(af[i].as<int>(), 500, 12000);
          if ((int)rf.size() >= 4) for (int i=0;i<4;i++) p->release_friction[i] = (uint16_t)constrain(rf[i].as<int>(), 500, 12000);
          if ((int)as.size() >= 4) for (int i=0;i<4;i++) p->apply_spring_mbar[i]   = (uint16_t)constrain(as[i].as<int>(), 0, 5000);
          if ((int)rs.size() >= 4) for (int i=0;i<4;i++) p->release_spring_mbar[i] = (uint16_t)constrain(rs[i].as<int>(), 0, 5000); }
        if (doc["tmax"].is<int>())    p->t_max_ref   = (uint16_t)constrain(doc["tmax"].as<int>(), 100, 1200);
        if (doc["tpsC"].is<float>())  p->tps_closed_v = doc["tpsC"].as<float>();
        if (doc["tpsW"].is<float>())  p->tps_wot_v    = doc["tpsW"].as<float>();
        if (doc["map0"].is<float>())  p->map_kpa_at_0v  = doc["map0"].as<float>();
        if (doc["mapV"].is<float>())  p->map_kpa_per_volt = doc["mapV"].as<float>();
        JsonArray fp = doc["fillp"].as<JsonArray>();
        JsonArray ft = doc["fillt"].as<JsonArray>();
        for (int i = 0; i < 4; i++) {
            if (fp.size() == 4) p->fill_p[i] = fp[i].as<uint8_t>();
            if (ft.size() == 4) p->fill_t[i] = ft[i].as<uint16_t>();
        }

    if (p->tps_wot_v <= p->tps_closed_v + 0.01f) return "TPS WOT must exceed closed voltage";
    return nullptr;
}
}
const char* validateCommand(JsonDocument& doc, ControlCommand& out) {
    const char* cmd = doc["cmd"] | "";
    if (!strcmp(cmd,"set_profile")) return profile(doc,out);
    if (!strcmp(cmd,"param.set") || !strcmp(cmd,"param.reset")) {
        TuneOverlay candidate;
        if (!strcmp(cmd,"param.set")) {
            *candidate.raw() = *tuneOverlay.raw();
            if (!number(doc["idx"],0,TuneOverlay::paramCount()-1)) return "Invalid parameter index";
            int idx = doc["idx"];
            const ParamDesc& p = TuneOverlay::paramDesc(idx);
            if (!number(doc["row"],0,p.rows-1) || !number(doc["col"],0,p.cols-1)
                || !number(doc["val"],p.vmin,p.vmax)) return "Invalid parameter cell or value";
            candidate.paramSet(idx,doc["row"],doc["col"],doc["val"]);
        }
        for (int r=0;r<TUNE_PAIRS;++r) for(int c=0;c<TUNE_TPS_PTS;++c)
            if(candidate.raw()->up_kmh[r][c] <= candidate.raw()->dn_kmh[r][c]) return "Upshift speed must exceed downshift speed";
        out.action=ControlAction::Tune; out.tune=*candidate.raw(); return nullptr;
    }
    if (!strcmp(cmd,"set_cells")) {
        JsonArrayConst a=doc["data"].as<JsonArrayConst>();
        if (a.size()!=ADAPT_CLASSES*ADAPT_SHIFTS*ADAPT_TBINS*3) return "Invalid adaptation array length";
        for(unsigned i=0;i<a.size();++i) if(!number(a[i],i%3==0?-5:-15,i%3==0?5:15)) return "Invalid adaptation value";
        out.action=ControlAction::Cells;
        for(unsigned i=0;i<a.size()/3;++i) out.cells[i]={a[i*3].as<int8_t>(),a[i*3+1].as<int8_t>(),a[i*3+2].as<int8_t>()};
        return nullptr;
    }
    if (!strcmp(cmd,"selector.atf")) {
        if (!doc["on"].is<bool>()) return "Boolean on required";
        out.action=ControlAction::AtfSelector; out.value=doc["on"].as<bool>(); return nullptr;
    }
    if (!strcmp(cmd,"test_mode")) {
        if(!doc["on"].is<bool>()) return "Boolean on required";
        out.action=ControlAction::TestMode; out.value=doc["on"].as<bool>(); return nullptr;
    }
    if (!strcmp(cmd,"test_prnd")) {
        const char* s=doc["v"] | "";
        if(strlen(s)!=1 || !strchr("PRND4321",s[0])) return "Invalid selector";
        out.action=ControlAction::Selector; out.value=s[0]; return nullptr;
    }
    if (!strcmp(cmd,"test_paddle") || !strcmp(cmd,"adapt_nudge")) {
        if(!number(doc["dir"],-1,1) || doc["dir"].as<int>()==0) return "Direction must be -1 or 1";
        out.action=!strcmp(cmd,"test_paddle")?ControlAction::Paddle:ControlAction::Nudge;
        out.value=doc["dir"]; return nullptr;
    }
    if (!strcmp(cmd,"test_sol") || !strcmp(cmd,"test_io")) {
        const char* id=doc["id"] | "";
        const char* ids[]={"y3","y5","y4","mpc","spc","tcc","rp","tq"};
        int n=0; for(int i=0;i<8;++i) if(!strcmp(id,ids[i])) n=i+1;
        if(!n || (!doc["on"].isNull() && !doc["on"].is<bool>())
            || (!doc["v"].isNull() && !number(doc["v"],0,100))) return "Invalid output command";
        out.action=ControlAction::Output; out.value=n;
        out.extra=(doc["on"] | true)?(doc["v"] | 50):-1; return nullptr;
    }
    if (!strcmp(cmd,"limp_reset")) { out.action=ControlAction::LimpReset; return nullptr; }
    if (!strcmp(cmd,"clear_dtcs")) { out.action=ControlAction::ClearDtcs; return nullptr; }
    return "Unknown command";
}
