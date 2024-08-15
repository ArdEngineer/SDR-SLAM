# About Resilience-VINS

## 仓库文件结构说明

要用好git仓库，首先要学会看每个仓库里面的markdown文件

这个文件就是一份说明书，文件怎么用、文件说明都在里面了

后期提交代码文档、还有其他杂七杂八的说明等都是以markdown文件形式提交

```
<DIR> project_resource->用来存放项目过程需要使用的一些包以及一些参考文献

<DIR> project_codes文件夹->后期用来存放我们项目自己写的代码，整个项目的工程代码都存在这个文件夹里面

<DIR> tutorials文件夹->文件夹里面的markdown文件给出了各种教程的链接，自己慢慢看

<DIR> reports->这个文件夹存放实验报告、采集的数据等等，方便后期写文章查看

<IGNORE FILE> .gitignore->这个文件别动就对了

<README FILE> README.md->仓库说明书

<EXCEL FILE> project_schedule.xlsx->工程进度/任务安排表，这个项目的开展阶段，阶段内的工作内容和负责人，各项工作时间和完成进度，后期通过此表记录工作情况
```

### 注意事项

每次提交到仓库的东西，每次commit的comment一定要好好写！不然别人都不知道你提交的是什么

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

每次开始工作之前必须用pull命令更新本地仓库，以使得文件树超前或与远程仓库进度相同

这么做的目的是为了减少工作成果合并时造成的分支冲突，免去手动解决冲突的麻烦

``` 
git pull
```

最后更新日期/时间：2024-8-15/23:43(UTC +8:00)