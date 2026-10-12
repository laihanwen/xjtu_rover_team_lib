#pragma once
#include <fstream>
#include <string>
#include <algorithm>
inline const std::string& pi_boot_identity() {
  static const std::string value=[] {
    std::ifstream stream("/proc/sys/kernel/random/boot_id");std::string id;std::getline(stream,id);
    if(id.size()!=36||!std::all_of(id.begin(),id.end(),[](char c){return
       (c>='a'&&c<='f')||(c>='0'&&c<='9')||c=='-';}))return std::string();
    return id;
  }();return value;
}
