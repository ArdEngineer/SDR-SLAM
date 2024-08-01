# Resilience-VINS

## 写在前面的话

关于大家配置项目环境需要的部分资源包或者代码脚本，我放在了project_resource文件夹下

大家配置环境需要参考的教程我统一写在了tutorials文件夹下的tutorials_web_resource.md文件里面

这个已经是比较完整的教程集合了，省去了大家重复寻找的过程，放在那的都是我精挑细选质量高的教程

我强烈建议按照里面的配置来（无数次试错总结出来的...

我们要把精力放在开发上面，而不要耗费过多精力在环境配置上面，这样能够最大程度节省大家时间

同时大家开发环境一致的话，后期可以省去很多跨平台部署项目所带来的麻烦，这样能够高效完成我们的项目开发~

### 总之，tutorials_web_resource.md这个文件很重要，碰到的问题80%都可以在那个文件所记录的教程找到答案

## git仓库简明使用教程和常用命令

#### GitHub仓库本地初始化

```
mkdir <DIR_NAME>
cd <DIR_NAME>

git init

git config --global user.name 'Your name'
git config --global user.email 'Your email address'
```

使用HTTP协议clone项目仓库到本地仓库（不推荐）

```
git clone https://github.com/ArdEngineer/Resilience-VINS.git
```

使用SSH协议克隆仓库到本地仓库（推荐，需要提前设置好SSH密钥）

```
git clone git@github.com:ArdEngineer/Resilience-VINS.git
```

后期大家都是先在本地进行修改工作，最后再由我来管理大家的分支合并

#### GitHub仓库设置SSH和关联远程仓库分支

Git设置SSH教程

https://www.runoob.com/git/git-remote-repo.html

但是GitHub官方将SSH加密算法由原来的SHA-1和rsa升级为了Ed25519，

所以参考下面的文章生成SSH key

https://blog.csdn.net/weixin_45462732/article/details/124415224

Git提交本地修改和提交到远程仓库指令

```
git add .
git commit -m "Write down your commit comment here"
git push
```

## 仓库文件结构说明

要用好git仓库，首先要学会看每个仓库里面的markdown文件

这个文件就是一份说明书，文件怎么用、文件说明都在里面了

后期提交代码文档、还有其他杂七杂八的说明等都是以markdown文件形式提交

不会用markdown的话建议学一下，很快的，会基本用法就行

### project_resource文件夹

用来存放项目过程需要使用的一些包以及一些参考文献

### project_codes文件夹

后期用来存放我们项目自己写的代码，整个项目的工程代码都存在这个文件夹里面

### tutorials文件夹

文件夹里面的markdown文件给出了各种教程的链接，自己慢慢看

### reports文件夹

这个文件夹存放实验报告、采集的数据等等，方便后期写文章查看

### .gitignore文件

这个文件别动就对了

### 其他注意事项

每次提交到仓库的东西，每次commit的comment一定要好好写，不然别人都不知道你提交的是什么