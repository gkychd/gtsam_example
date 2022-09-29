file(GLOB ex_headers "include/*.h") #打包需要生成库文件的h文件
install(FILES ${ex_headers} DESTINATION test/gtsamexamples) #确定文件安装的位置

include_directories(include)

gtsamAddExamplesGlob("src/*.cpp" "" "${PROJECT_NAME}")  #这里表示生成可执行文件，每个cpp文件为一个可执行文件

file(GLOB ex_srcs "*.cpp") #打包需要生成库文件的cpp文件

message(STATUS "ex_srcs: ${ex_srcs}")
message(STATUS "ex_headers: ${ex_headers}")

add_library(${PROJECT_NAME} SHARED ${ex_srcs} ${ex_headers})     #将functions.cpp生成库函数 SHARED表示生成动态链接库

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)

