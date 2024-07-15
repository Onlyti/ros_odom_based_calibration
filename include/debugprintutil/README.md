# DebugPrintUtil
DebugPrint uilt header
콘솔에 프린트 하기 위한 도구입니다.

어떤 종류(Info, Warn, Error)의 메시지인지, 출력시 시간은 언제인지, 어떤 값을 표시할지(옵션입니다.), 어떤 메시지를 표시할지, 또 해당 메시지의 색까지 설정할 수 있는 프린트 도구입니다.

표시 예
```
[PRTIN_OPTION][CURRENT_TIME] (VALUE=optional) (MESSAGE_TO_SHOW)
```
# Install

Clone this repository in your project's include path

해당 래포지토리를 사용하시는 프로젝트의 include폴더에 clone해 주십시오.

# How to use
## Inputs
There are three input option

**debug_info**: Some string to print

**value** : Some value to show. Any type is ok if can be printed by std::cout. ex) int, double, bool, string, ...

**measure_time**: Use with MeasureComputingTime class. DebugPrinter print time until this print code and restart timer

**flag**  : Flag to print 

**color** : Color of text. It is optinal. Default option is defined

---
## Forms

There are three forms each message type **(info, warn, error)**
1. void DebugPrintInfo(std::string debug_info,TYPE value, bool flag, Color color=RESET)

>>>Print debug message (debug_info) and value, if flag is true. 

2. void DebugPrintInfo(std::string debug_info, bool flag, Color color=RESET);

>>>Print debug message (debug_info), if flag is true. 

3. void DebugPrintInfoTime(std::string debug_info,MeasureComputingTime &measure_time, bool flag, Color color=RESET);

>>>Print computing time with measurea_time class and debug message (debug_info), if flag is true. 

## Code example

### Origin form
```
void DebugPrintInfo(std::string debug_info,TYPE value, bool flag, Color color=RESET);
void DebugPrintInfo(std::string debug_info, bool flag, Color color=RESET);
void DebugPrintInfoTime(std::string debug_info,MeasureComputingTime &measure_time, bool flag, Color color=RESET);
```

### Example
```
DebugPrintInfo("hello world", 3.14, true);
```
>[Info][124.404222] hello world3.14
```
DebugPrintWarn("hello world", true, YELLOW);
```
><span style="color:yellow">[Warn][124.404222] hello world (The text will show yellow)</span>
```
DebugPrintErrorTime("hello world", measure_time, true, BLUE);
```
><span style="color:blue">[Error][124.404222] [Time diff: 0.02004(s)] hello world (The text will show blue)</span>


## MeasureComputingTime

MeasureComputingTime class will check the computing time sience class initialized (created);
### Example
```
MeasureComputingTime measure_time; // start computing time
SomeProcess(); // computing time is 0.5 sec
DebugPrintInfo("debug_message", measure_time, true);
```

then show the text like this
> [Info][124.404222] [Time diff: 0.50000(s)] debug_message

# Development behind
해당 툴을 개발한 시기는 2022년 2월즈음 아주 추운 날이었습니다.

당시 현대자동차 노면인지 프로젝트를 열심히 마감하기 위해 ros cpp에서 개발중이었고 디버깅을 위해 cout을 열심히 치고 있었습니다.

값을 표시하거나 시간을 표시하거나 또 디버그 플레그를 통해 프린트를 없에기 위해 코드를 열심히 수정하고 있었죠.

너무 귀찮았던 나머지 해당 해더파일을 만들게 되었습니다.

하는김에 색도 알록달록하게 나오게 만들었죠.