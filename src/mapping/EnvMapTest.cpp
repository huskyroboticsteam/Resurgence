#include "EnvMapTest.h"
#include "EnvMap.h"

int main(int argc, const char * argv[]) {
    EMT_TESTSET_BEGIN("EnvMap tests")

    EMT_TEST_BEGIN("EnvMap is constructible")
    EMT_ASSERT_NOTHROW_BEGIN
    EnvMap map;
    EMT_ASSERT_NOTHROW_END
    EMT_TEST_END

    EMT_TEST_BEGIN("MapObstacles can be created and referenced")
    EnvMap map;

    // test that we can store obstacles
    MapObstacle proto_obst;
    proto_obst.position.x = 1.0f;
    proto_obst.position.y = - 1.0f;
    size_t obst_uid_1 = map.newObstacleUID(proto_obst);

    proto_obst.position.x = - 1.5f;
    proto_obst.position.y = - 6.0f;
    size_t obst_uid_2 = map.newObstacleUID(proto_obst);

    std::shared_ptr<MapObstacle> obst_1_ptr = map.getObstacle(obst_uid_1);
    std::shared_ptr<MapObstacle> obst_2_ptr = map.getObstacle(obst_uid_2);

    EMT_ASSERT_NOT_EQ(obst_1_ptr, nullptr);
    EMT_ASSERT_NOT_EQ(obst_2_ptr, nullptr);
    
    EMT_ASSERT_EQ(obst_1_ptr->position.x, 1.0f);
    EMT_ASSERT_EQ(obst_1_ptr->position.y, - 1.0f);
    EMT_ASSERT_EQ(obst_2_ptr->position.x, - 1.5f);
    EMT_ASSERT_EQ(obst_2_ptr->position.y, - 6.0f);

    EMT_TEST_END

    EMT_TESTSET_END

    return 0;
}
