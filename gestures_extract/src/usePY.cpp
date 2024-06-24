#include <iostream>
#include <Python.h>

int main() {
    // 初始化，载入python的扩展模块
    Py_Initialize();
    if(!Py_IsInitialized()) {
        std::cout << "Python init failed!" << std::endl;
        return -1;
    }

    // PyRun_SimpleString 为宏，执行一段python代码
    // 导入当前路径
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('./')");
    PyRun_SimpleString("import random");
    PyRun_SimpleString("import string");
    PyRun_SimpleString("print(''.join(random.sample(string.ascii_letters + string.digits, 10)))");

    PyObject *pModule = NULL;
    PyObject *pDict = NULL;
    PyObject *pFunc = NULL;
    PyObject *pArgs = NULL;
    PyObject *pRet = NULL;

    // 使用PyObject* pModule来存储导入的.py文件模块
    pModule = PyImport_ImportModule("test");
    if(!pModule) {
        std::cout << "Load test.py failed!" << std::endl;
        return -1;
    }

    // 使用PyObject* pDict来存储导入模块中的方法字典
    pDict = PyModule_GetDict(pModule);
    if(!pDict) {
        std::cout << "Can't find dict in test!" << std::endl;
        return -1;
    }

    // 获取方法
    pFunc = PyDict_GetItemString(pDict, "mutstr");
    if(!pFunc || !PyCallable_Check(pFunc)) {
        std::cout << "Can't find function!" << std::endl;
        return -1;
    }

    /*
    向Python传参数是以元组（tuple）的方式传过去的，
    因此我们实际上就是构造一个合适的Python元组就
    可以了，要用到PyTuple_New，Py_BuildValue，PyTuple_SetItem等几个函数
    */
    pArgs = PyTuple_New(1);

    //  PyObject* Py_BuildValue(char *format, ...) 
    //  把C++的变量转换成一个Python对象。当需要从 
    //  C++传递变量到Python时，就会使用这个函数。此函数 
    //  有点类似C的printf，但格式不同。常用的格式有 
    //  s 表示字符串， 
    //  i 表示整型变量， 如Py_BuildValue("ii",123,456)
    //  f 表示浮点数， 
    //  O 表示一个Python对象
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("i", 60));

    // 调用python的mutstr函数
    pRet = PyObject_CallObject(pFunc, pArgs);
    if (pRet != NULL) {
        char* retstr = NULL;
        PyArg_Parse(pRet, "s", &retstr);
        std::cout << "-- c++ is: " << retstr << std::endl;
    }

    // 调用python的mutnum函数
    pFunc = PyDict_GetItemString(pDict, "mutnum");
    if(!pFunc || !PyCallable_Check(pFunc)) {
        std::cout << "Can't find function!" << std::endl;
        return -1;
    }
    pRet = PyObject_CallObject(pFunc, pArgs);
    if (pRet != NULL) {
        int retnum = 0;
        PyArg_Parse(pRet, "i", &retnum);
        std::cout << "-- c++ is: " << retnum << std::endl;
    }   

    // 清理python对象
    if(pArgs) {
        Py_DECREF(pArgs);
    }
    if(pModule) {
        Py_DECREF(pModule);
    }

    //关闭python调用
    Py_Finalize();

    return 0;
}