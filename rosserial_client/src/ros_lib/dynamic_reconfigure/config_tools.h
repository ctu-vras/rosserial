#ifndef __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__
#define __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__

#include <string>
#include <utility>
#include <vector>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Group.h>

namespace dynamic_reconfigure
{

class ConfigTools
{
public:
  static std::pair<dynamic_reconfigure::BoolParameter*, uint32_t&> getVectorForType(dynamic_reconfigure::Config &set, const bool /*val*/)
  {
    return {set.bools, set.bools_length};
  }

  static std::pair<dynamic_reconfigure::IntParameter*, uint32_t&> getVectorForType(dynamic_reconfigure::Config &set, const int /*val*/)
  {
    return {set.ints, set.ints_length};
  }

  static std::pair<dynamic_reconfigure::StrParameter*, uint32_t&> getVectorForType(dynamic_reconfigure::Config &set, const std::string& /*val*/)
  {
    return {set.strs, set.strs_length};
  }

  static std::pair<dynamic_reconfigure::DoubleParameter*, uint32_t&> getVectorForType(dynamic_reconfigure::Config &set, const double /*val*/)
  {
    return {set.doubles, set.doubles_length};
  }

  static const std::pair<dynamic_reconfigure::BoolParameter*, const uint32_t&> getVectorForType(const dynamic_reconfigure::Config &set, const bool /*val*/)
  {
    return {set.bools, set.bools_length};
  }

  static const std::pair<dynamic_reconfigure::IntParameter*, const uint32_t&> getVectorForType(const dynamic_reconfigure::Config &set, const int /*val*/)
  {
    return {set.ints, set.ints_length};
  }

  static const std::pair<dynamic_reconfigure::StrParameter*, const uint32_t&> getVectorForType(const dynamic_reconfigure::Config &set, const std::string& /*val*/)
  {
    return {set.strs, set.strs_length};
  }

  static const std::pair<dynamic_reconfigure::DoubleParameter*, const uint32_t&> getVectorForType(const dynamic_reconfigure::Config &set, const double /*val*/)
  {
    return {set.doubles, set.doubles_length};
  }

  static void appendParameter(Config &set, const char* name, const bool &val)
  {
    set.bools_length++;
    BoolParameter* old = set.bools;
    set.bools = new BoolParameter[set.bools_length];
    for (size_t i = 0; i < set.bools_length - 1; ++i)
      set.bools[i] = old[i];
    set.bools[set.bools_length - 1].name = name;
    set.bools[set.bools_length - 1].value = val;
    delete[] old;
  }

  static void appendParameter(Config &set, const char* name, const int &val)
  {
    set.ints_length++;
    IntParameter* old = set.ints;
    set.ints = new IntParameter[set.ints_length];
    for (size_t i = 0; i < set.ints_length - 1; ++i)
      set.ints[i] = old[i];
    set.ints[set.ints_length - 1].name = name;
    set.ints[set.ints_length - 1].value = val;
    delete[] old;
  }

  static void appendParameter(Config &set, const char* name, const double &val)
  {
    set.doubles_length++;
    DoubleParameter* old = set.doubles;
    set.doubles = new DoubleParameter[set.doubles_length];
    for (size_t i = 0; i < set.doubles_length - 1; ++i)
      set.doubles[i] = old[i];
    set.doubles[set.doubles_length - 1].name = name;
    set.doubles[set.doubles_length - 1].value = val;
    delete[] old;
  }

  static void appendParameter(Config &set, const char* name, const std::string &val)
  {
    set.strs_length++;
    StrParameter* old = set.strs;
    set.strs = new StrParameter[set.strs_length];
    for (size_t i = 0; i < set.strs_length - 1; ++i)
      set.strs[i] = old[i];
    set.strs[set.strs_length - 1].name = name;
    set.strs[set.strs_length - 1].value = val.c_str();
    delete[] old;
  }

  template <class VT, class T>
  static bool getParameter(const std::pair<VT*, const uint32_t&> &vec, const char* name, T &val)
  {
    for (size_t i = 0; i < vec.second; ++i)
      if (strcmp(vec.first[i].name, name) == 0)
      {
        val = vec.first[i].value;
        return true;
      }
    return false;
  }

  template <class T>
  static bool getParameter(const dynamic_reconfigure::Config &set, const char* name, T &val)
  {
    return getParameter(getVectorForType(set, val), name, val);
  }

  template<class T>
  static void appendGroup(dynamic_reconfigure::Config &set, const char* name, int id, int parent, const T &val)
  {
    dynamic_reconfigure::GroupState msg;
    msg.name = name;
    msg.id= id;
    msg.parent = parent;
    msg.state = val.state;
    set.groups_length++;
    GroupState* old = set.groups;
    set.groups = new GroupState[set.groups_length];
    for (size_t i = 0; i < set.groups_length - 1; ++i)
      set.groups[i] = old[i];
    set.groups[set.groups_length - 1] = msg;
    delete[] old;
  }

  template<class T>
  static bool getGroupState(const dynamic_reconfigure::Config &msg, const char* name, T &val)
  {
    for(size_t i = 0; i < msg.groups_length; ++i)
      if(strcmp(msg.groups[i].name, name) == 0)
      {
        val.state = msg.groups[i].state;
        return true;
      }
    return false;
  }

  static int size(const dynamic_reconfigure::Config &msg)
  {
    return msg.bools_length + msg.doubles_length + msg.ints_length + msg.strs_length;
  }

  static void clear(dynamic_reconfigure::Config &msg)
  {
    msg.bools_length = 0;
    delete[] msg.bools;
    msg.bools = nullptr;

    msg.ints_length = 0;
    delete[] msg.ints;
    msg.ints = nullptr;

    msg.strs_length = 0;
    delete[] msg.strs;
    msg.strs = nullptr;

    msg.doubles_length = 0;
    delete[] msg.doubles;
    msg.doubles = nullptr;

    msg.groups_length = 0;
    delete[] msg.groups;
    msg.groups = nullptr;

  }
};

}

#endif
