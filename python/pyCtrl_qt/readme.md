# pyqt

## 工具
### qtdesigner 
```
pip install PyQt5-tools -i https://pypi.douban.com/simple
```
安装完成后需到安装目录下找到可执行文件

### pyGUI
```
pip install PyQt5
```
## 生成exe工具
```
pip install PyInstaller
```

## 工具使用

### 启动qtdesigner
\Lib\site-packages\PyQt5\designer.exe

### ui到python
pyuic5 -o a.py a.ui
参考：https://blog.csdn.net/dyxcome/article/details/87977545

### 打包发布
pyinstaller -F -w test.py
参考：https://zhuanlan.zhihu.com/p/52654565


