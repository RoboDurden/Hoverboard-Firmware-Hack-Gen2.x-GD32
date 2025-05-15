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

add_executable(test_commutation test_commutation.c ../Src/blockCommutation.c)
target_link_libraries(test_commutation cmocka)

enable_testing()
add_test(NAME commutation_tests COMMAND test_commutation)