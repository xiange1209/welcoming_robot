/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <string>
#include <type_traits>
#include <utility>

namespace frontier_exploration_ros2::compat
{

template<typename T, typename = void>
struct HasNav2ErrorCodeField : std::false_type {};

template<typename T>
struct HasNav2ErrorCodeField<T, std::void_t<decltype(std::declval<const T &>().error_code)>>
: std::true_type {};

template<typename T, typename = void>
struct HasNav2ErrorMsgField : std::false_type {};

template<typename T>
struct HasNav2ErrorMsgField<T, std::void_t<decltype(std::declval<const T &>().error_msg)>>
: std::true_type {};

template<typename T>
int extractNav2ResultErrorCode(const T & result)
{
  if constexpr (HasNav2ErrorCodeField<T>::value) {
    return result.error_code;
  }
  return 0;
}

template<typename T>
std::string extractNav2ResultErrorMessage(const T & result)
{
  if constexpr (HasNav2ErrorMsgField<T>::value) {
    return result.error_msg;
  }
  return "";
}

}  // namespace frontier_exploration_ros2::compat
