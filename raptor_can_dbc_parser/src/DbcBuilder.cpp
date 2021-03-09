// Copyright (c) 2021 New Eagle, All rights reserved.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <raptor_can_dbc_parser/DbcBuilder.hpp>
#include <raptor_can_dbc_parser/DbcMessage.hpp>
#include <raptor_can_dbc_parser/DbcSignal.hpp>

#include <cstring>
#include <iostream>
#include <map>
#include <string>

namespace NewEagle
{
DbcBuilder::DbcBuilder()
{
  MessageToken = std::string("BO_");
  SignalToken = std::string("SG_");
  CommentToken = std::string("CM_");
  EnumValueToken = std::string("VAL_");
  AttributeToken = std::string("BA_");
  SignalValueTypeToken = std::string("SIG_VALTYPE_");
}

NewEagle::Dbc DbcBuilder::NewDbc(const std::string & dbcFile)
{
  NewEagle::Dbc dbc;

  std::ifstream f(dbcFile);
  std::string line;
  if (!f.is_open()) {
    std::string error_msg("Unable to open DBC file " + dbcFile);
    throw std::runtime_error(error_msg);
  }

  uint32_t lineNumber = 0;

  NewEagle::DbcMessage currentMessage;

  while (std::getline(f, line, '\n')) {
    lineNumber++;
    NewEagle::LineParser parser(line);
    std::string errItemName("");

    std::string identifier;
    try {
      identifier = parser.ReadCIdentifier();
    } catch (std::exception & ex) {
      identifier = std::string();
    }

    if (!MessageToken.compare(identifier)) {
      try {
        currentMessage = ReadMessage(parser);
        dbc.AddMessage(currentMessage);
        errItemName = "message " + currentMessage.GetName();
      } catch (LineParserExceptionBase & exlp) {
        std::string error_msg(
          "Tried to add " + errItemName +
          ". Got Line Parser Exception Base error.");
        throw std::runtime_error(error_msg);
      } catch (std::exception & ex) {
        std::string error_msg(
          "Tried to add " + errItemName +
          ". Got Standard Exception error.");
        throw std::runtime_error(error_msg);
      }
    } else if (!SignalToken.compare(identifier)) {
      try {
        NewEagle::DbcSignal signal = ReadSignal(parser);
        errItemName = "message " + currentMessage.GetName() +
          ", signal " + signal.GetName();

        NewEagle::DbcMessage * msg = dbc.GetMessage(currentMessage.GetName());
        msg->AddSignal(signal.GetName(), signal);
      } catch (LineParserExceptionBase & exlp) {
        std::string error_msg(
          "Tried to add " + errItemName +
          ". Got Line Parser Exception Base error.");
        throw std::runtime_error(error_msg);
      } catch (std::exception & ex) {
        std::string error_msg(
          "Tried to add " + errItemName +
          ". Got Standard Exception error.");
        throw std::runtime_error(error_msg);
      }
    } else if (!CommentToken.compare(identifier)) {
      try {
        std::string token = parser.ReadCIdentifier();
        errItemName = "CI Identifier " + token;

        if (!MessageToken.compare(token)) {
          NewEagle::DbcMessageComment dbcMessageComment = ReadMessageComment(parser);

          std::map<std::string, NewEagle::DbcMessage>::iterator it;
          int32_t ttt = 0;

          for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it) {
            ttt++;
            if (it->second.GetRawId() == dbcMessageComment.Id) {
              it->second.SetComment(dbcMessageComment);
              break;
            }
          }
        } else if (!SignalToken.compare(token)) {
          NewEagle::DbcSignalComment dbcSignalComment = ReadSignalComment(parser);

          std::map<std::string, NewEagle::DbcMessage>::iterator it;
          for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it) {
            if (it->second.GetRawId() == dbcSignalComment.Id) {
              DbcMessage msg = it->second;
              std::map<std::string, NewEagle::DbcSignal>::iterator its;

              for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its) {
                if (its->second.GetName() == dbcSignalComment.SignalName) {
                  its->second.SetComment(dbcSignalComment);
                  break;
                }
              }
            }
          }
        }
      } catch (LineParserExceptionBase & exlp) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Line Parser Exception Base error.");
        throw std::runtime_error(error_msg);
      } catch (std::exception & ex) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Standard Exception error.");
        throw std::runtime_error(error_msg);
      }
    } else if (!AttributeToken.compare(identifier)) {
      try {
        NewEagle::DbcAttribute dbcAttribute = ReadAttribute(parser);
        errItemName = "DBC Attribute " + dbcAttribute.AttributeName;

        if (dbc.GetMessageCount() > 0) {
          std::map<std::string, NewEagle::DbcMessage>::iterator it;
          for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it) {
            if (it->second.GetRawId() == dbcAttribute.Id) {
              std::map<std::string, NewEagle::DbcSignal>::iterator its;
              DbcMessage msg = it->second;
              for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its) {
                if (its->second.GetName() == dbcAttribute.SignalName) {
                  NewEagle::DbcSignal sig = its->second;

                  double gain = sig.GetGain();
                  double offset = sig.GetOffset();

                  double f = 0.0;

                  std::stringstream ss;
                  ss << dbcAttribute.Value;
                  ss >> f;

                  double val = gain * f + offset;
                  sig.SetInitialValue(val);
                  break;
                }
              }
            }
          }
        }
      } catch (LineParserExceptionBase & exlp) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Line Parser Exception Base error.");
        throw std::runtime_error(error_msg);
      } catch (std::exception & ex) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Standard Exception error.");
        throw std::runtime_error(error_msg);
      }
    } else if (!EnumValueToken.compare(identifier)) {
      // TODO(NewEagle): Empty for now.
    } else if (!SignalValueTypeToken.compare(identifier)) {
      try {
        NewEagle::DbcSignalValueType dbcSignalValueType = ReadSignalValueType(parser);
        errItemName = "DBC Signal Value Type " + dbcSignalValueType.SignalName;

        if (dbc.GetMessageCount() > 0) {
          std::map<std::string, NewEagle::DbcMessage>::iterator it;
          for (it = dbc.GetMessages()->begin(); it != dbc.GetMessages()->end(); ++it) {
            if (it->second.GetRawId() == dbcSignalValueType.Id) {
              std::map<std::string, NewEagle::DbcSignal>::iterator its;
              DbcMessage msg = it->second;
              for (its = msg.GetSignals()->begin(); its != msg.GetSignals()->end(); ++its) {
                if (its->second.GetName() == dbcSignalValueType.SignalName) {
                  NewEagle::DbcSignal sig = its->second;
                  sig.SetDataType(dbcSignalValueType.Type);
                  break;
                }
              }
            }
          }
        }
      } catch (LineParserExceptionBase & exlp) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Line Parser Exception Base error.");
        throw std::runtime_error(error_msg);
      } catch (std::exception & ex) {
        std::string error_msg(
          "Tried to read " + errItemName +
          ". Got Standard Exception error.");
        throw std::runtime_error(error_msg);
      }
    }
  }
  std::cout << "DBC Size: " << dbc.GetMessageCount() << std::endl;
  return dbc;
}
}  // namespace NewEagle
