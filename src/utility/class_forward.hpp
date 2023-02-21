#pragma once

#include <memory>

#define PLANNER_DECLARE_PTR(Name, Type)                      \
  typedef std::shared_ptr<Type> Name##Ptr;                   \
  typedef std::shared_ptr<const Type> Name##ConstPtr;         \
  typedef std::unique_ptr<Type> Name##UPtr;

#define PLANNER_CLASS_FORWARD(C)                             \
  class C;                                                   \
  PLANNER_DECLARE_PTR(C, C)

#define PLANNER_STRUCT_FORWARD(C)                            \
  struct C;                                                  \
  PLANNER_DECLARE_PTR(C, C)
