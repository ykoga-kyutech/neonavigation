add_rostest_gtest(test_safety_limiter
  test/safety_limiter_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter safety_limiter)

add_rostest_gtest(test_safety_limiter_collision
  test/safety_limiter_collision_rostest.test
  src/test_safety_limiter_collision.cpp
)
target_link_libraries(test_safety_limiter_collision ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter_collision safety_limiter_simple)

add_rostest_gtest(test_safety_limiter_compat
  test/safety_limiter_compat_rostest.test
  src/test_safety_limiter.cpp
)
target_link_libraries(test_safety_limiter_compat ${catkin_LIBRARIES})
add_dependencies(test_safety_limiter_compat safety_limiter)
