#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if(argc != 3){
        std::cout << "用法: rosrun cr5_car_moveit crop_floor <输入图.bt> <输出图.bt>\n";
        return -1;
    }

    std::cout << "正在加载八叉树: " << argv[1] << " ...\n";
    octomap::OcTree* tree = new octomap::OcTree(argv[1]);
    
    // 设定切除高度 (0.15米，即底盘和地面的厚度，可根据需要微调)
    double min_z = 0.1; 
    std::vector<octomap::OcTreeKey> keys_to_delete;

    // 遍历所有叶子节点，找出高度低于 min_z 的地板节点
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end=tree->end_leafs(); it!= end; ++it) {
        if(it.getZ() < min_z) {
            keys_to_delete.push_back(it.getKey());
        }
    }

    std::cout << "找到 " << keys_to_delete.size() << " 个地板节点，正在切除...\n";
    for(auto key : keys_to_delete) {
        tree->deleteNode(key);
    }

    tree->updateInnerOccupancy(); // 更新树的内部状态
    tree->writeBinary(argv[2]);   // 保存为新的文件

    std::cout << "手术成功！去除了地板的新地图已保存至: " << argv[2] << "\n";
    delete tree;
    return 0;
}