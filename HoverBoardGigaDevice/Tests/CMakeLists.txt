cmake_minimum_required(VERSION 3.10)
project(commutation_tests C)

include(FetchContent)

FetchContent_Declare(
  cmocka
  GIT_REPOSITORY https://git.cryptomilk.org/projects/cmocka.git
  GIT_TAG        cmocka-1.1.7
)

FetchContent_MakeAvailable(cmocka)

include_directories(
    ${cmocka_SOURCE_DIR}/include
    ../Inc
)

add_executable(block_commutation test_block_commutation.c ../Src/blockCommutation.c)
target_link_libraries(block_commutation cmocka)
target_compile_definitions(block_commutation PRIVATE COMMUTATION_MODE=0)

add_executable(svm_commutation test_svm_commutation.c ../Src/svmCommutation.c)
target_link_libraries(svm_commutation cmocka)
target_compile_definitions(svm_commutation PRIVATE COMMUTATION_MODE=1 PHASE_ADVANCE_AT_MAX_PWM_DEGREES=0)

#enable_testing()
#add_test(NAME block_commutation COMMAND block_commutation)
#add_test(NAME svm_commutation COMMAND svm_commutation)