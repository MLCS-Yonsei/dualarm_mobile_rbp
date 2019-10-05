#ifndef UTILS_H_
#define UTILS_H_

#include <string.h>
#include <sstream>
#include <iostream>


namespace std
{

void LogInfo(string message)
{
  cout << message << endl;
}


template <typename T>
std::string dec_to_hex(T dec, int NbofByte)
{
  std::stringstream stream_HL;
  string s, s_LH;
  stream_HL << std::setfill ('0') << std::setw(sizeof(T)*2) <<std::hex << dec;
  s = stream_HL.str();
  for(int i = 0; i < NbofByte; i++)
  {
    s_LH.append(s.substr(2*(NbofByte-1-i),2));
  }
  return s_LH;
}


int hexarray_to_int(unsigned char *buffer, int length)
{
  int hextoint = 0;
  for(int i = 0; i < length; i++)
  {
    hextoint += (buffer[i] << 8*i);
  }
  return hextoint;
}


double hexarray_to_double(unsigned char *buffer, int length)
{
  double hextodouble = 0.0;
  hextodouble = double(hexarray_to_int(buffer, length))
  return hextodouble;
}


std::string stringappend(string a, string b)
{
  string s;
  s.append(a);
  s.append(b);
  return s;
}


} // end namespace std

#endif // _UTILS_H_