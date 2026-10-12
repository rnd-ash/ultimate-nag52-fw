#include <cassert>
#include <cstdio>
struct Info {int pressure_multi_spc_int;}; struct Sid {Info inf;};
struct InterpType {static constexpr int Linear=0;};
float interpolate_float(float v,float a,float b,float x,float y,int){if(v<x)return a;if(v>y)return b;return a+(v-x)*(b-a)/(y-x);}
#include "production.h"
int main(){
 Sid sid{{1000}};assert(adjusted(990,60,8,&sid)==1020);
 assert(adjusted(1990,60,8,&sid)==2000);assert(adjusted(-1990,-200,8,&sid)==-2000);
 sid.inf.pressure_multi_spc_int=500;assert(adjusted(490,60,8,&sid)==520);assert(adjusted(990,60,8,&sid)==1000);
 sid.inf.pressure_multi_spc_int=1500;assert(adjusted(1490,60,8,&sid)==1520);assert(adjusted(2990,60,8,&sid)==3000);
 puts("PASS: incremental SPC adaptation and symmetric scaled limits");
}
