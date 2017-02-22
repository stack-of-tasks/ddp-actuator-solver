ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(test)

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)

FILE(
    GLOB_RECURSE
    source_files
    include/*
    src/*
)

ADD_EXECUTABLE(main test/main.cpp ${source_files})
ADD_EXECUTABLE(mainVIA test/mainVIA.cpp ${source_files})
ADD_EXECUTABLE(mainTorque test/mainTorque.cpp ${source_files})
ADD_EXECUTABLE(mainMPC test/mainMPC.cpp ${source_files})
ADD_EXECUTABLE(mainMPCVIA test/mainMPCVIA.cpp ${source_files})
ADD_EXECUTABLE(mainTorqueMPC test/mainTorqueMPC.cpp ${source_files})
ADD_EXECUTABLE(testModel test/testModel.cpp ${source_files})
ADD_EXECUTABLE(testAsserv test/testAsserv.cpp ${source_files})
ADD_EXECUTABLE(benchControl test/benchControl.cpp ${source_files} test/BenchCom.cpp test/BenchCom.h)
ADD_EXECUTABLE(benchControlMPC test/benchControlMPC.cpp ${source_files} test/BenchCom.cpp test/BenchCom.h)
ADD_EXECUTABLE(simulateBench test/simulateBench.cpp ${source_files} test/BenchCom.cpp test/BenchCom.h)
ADD_EXECUTABLE(benchTorqueControl test/benchTorqueControl.cpp ${source_files} test/BenchCom.cpp test/BenchCom.h)
ADD_EXECUTABLE(mainLQR test/mainLQR.cpp ${source_files})
ADD_EXECUTABLE(dccontrol test/dccontrol.cpp ${source_files})

TARGET_LINK_LIBRARIES(main ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(mainVIA ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(mainTorque ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(mainMPC ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(mainMPCVIA ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(mainTorqueMPC ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(testModel ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(testAsserv ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(benchControl ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(benchControlMPC ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(simulateBench ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(benchTorqueControl ${qpOASES_LIBRARIES})
TARGET_LINK_LIBRARIES(dccontrol ${qpOASES_LIBRARIES})