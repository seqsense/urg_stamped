/*
 * Copyright 2018 The urg_stamped Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SCIP2_DECODE_H
#define SCIP2_DECODE_H

#include <string>

namespace scip2
{
class DecoderRemain
{
public:
  uint64_t buf_;
  uint32_t chars_;

  DecoderRemain()
    : buf_(0)
    , chars_(0)
  {
  }
  DecoderRemain(const uint64_t& buf, const uint32_t& chars)
    : buf_(buf)
    , chars_(chars)
  {
  }
};

template <int L>
class Decoder
{
protected:
  const std::string::const_iterator begin_;
  const std::string::const_iterator end_;
  const DecoderRemain remain_;

public:
  class Iterator
  {
  protected:
    std::string::const_iterator pos_;
    std::string::const_iterator end_;
    DecoderRemain remain_;

  public:
    Iterator(
        const std::string::const_iterator& pos,
        const std::string::const_iterator& end,
        const DecoderRemain& remain = DecoderRemain())
      : pos_(pos)
      , end_(end)
      , remain_(remain)
    {
    }
    bool operator==(const Iterator& it) const
    {
      return pos_ == it.pos_;
    }
    bool operator!=(const Iterator& it) const
    {
      return !operator==(it);
    }
    void operator++()
    {
      pos_ += L - remain_.chars_;
      if (pos_ + L > end_)
      {
        remain_ = DecoderRemain(operator*(), end_ - pos_);
        pos_ = end_;
      }
      else
      {
        remain_ = DecoderRemain();
      }
    }
    DecoderRemain getRemain() const
    {
      return remain_;
    }
    const uint64_t operator*()
    {
      std::string::const_iterator pos(pos_);
      uint64_t buf(remain_.buf_);
      for (size_t i = 0; i < L - remain_.chars_; ++i)
      {
        if (pos == end_)
          break;
        buf = buf << 6;
        buf |= (static_cast<uint8_t>(*pos) - 0x30);
        ++pos;
      }
      return buf;
    }
  };

  explicit Decoder(const std::string& line, const DecoderRemain& remain = DecoderRemain())
    : begin_(line.begin())
    , end_(line.end())
    , remain_(remain)
  {
  }
  Iterator begin()
  {
    return Iterator(begin_, end_, remain_);
  }
  Iterator end()
  {
    return Iterator(end_, end_);
  }
  uint8_t getChecksum() const
  {
    uint8_t sum = 0;
    for (std::string::const_iterator it = begin_; it != end_; ++it)
    {
      sum += *it;
    }
    return sum;
  }
};
}  // namespace scip2

#endif  // SCIP2_DECODE_H
