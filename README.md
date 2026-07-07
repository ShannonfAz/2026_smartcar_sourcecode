基于新架构图像的智能车代码，配好和正常逐飞方案一致的环境就能跑

请使用110°无畸变镜头，并将镜头压低，确保刚好不能同时拍到环岛的上拐点和下拐点，本算法基于此外部环境下调试

控制很烂，目前1.5m/s

这些文件夹/文件应该在LS2k0300_LIBRARY/Seekfree_LS2K0300_Opensource_Library下

如需clone，请备份后删除LS2k0300_LIBRARY/Seekfree_LS2K0300_Opensource_Library下的内容

然后在LS2k0300_LIBRARY/Seekfree_LS2K0300_Opensource_Library里

```bash
git clone https://github.com/ShannonfAz/2026_smartcar_sourcecode.git
```

哦对了，记得把编码器改成你车上的编码器种类，我这是方向编码器，你那可不一定

---

更新日志：

2026.6.4 建仓库

2026.7.7 精简道路分类模型，平均推理时间从6ms降为250ns