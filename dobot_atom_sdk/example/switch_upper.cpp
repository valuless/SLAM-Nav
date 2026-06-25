#include "rpc/algs_rpc_client.h"
#include <iostream>
using namespace Atom;
using namespace std;

int main(int argc, char* argv[]) {
    // 用法检查
    if (argc != 2) {
        cout << "用法: " << endl;
        cout << "  ./switch_upper on   开启上肢控制权" << endl;
        cout << "  ./switch_upper off  关闭上肢控制权" << endl;
        return 1;
    }

    // 创建RPC客户端
    AlgsRpcClient client;

    // 解析命令
    string cmd = argv[1];
    RpcErrorCode ret;

    if (cmd == "on") {
        ret = client.SwitchUpperLimbControl(true);
        if (ret == Atom::RpcErrorCode::SUCCESS)
            cout << "✅ 上肢控制权已开启" << endl;
    } 
    else if (cmd == "off") {
        ret = client.SwitchUpperLimbControl(false);
        if (ret == Atom::RpcErrorCode::SUCCESS)
            cout << "✅ 上肢控制权已关闭" << endl;
    }

    if (ret != Atom::RpcErrorCode::SUCCESS) {
        cout << "❌ 失败，错误码: " << ret << endl;
        return 1;
    }

    return 0;
}
