# Git & GitHub 项目初始化与提交实战教程

本教程专门为你解答：**如何将本地 `/home/freebird/AntigraProj/Arm_Grabbing` 机械臂控制项目上传并展示在 GitHub 上。**

---

## 1. 为什么需要 `.gitignore`？
在 ROS 2 项目编译后，会生成 `build/`、`install/`、`log/` 三个几百 MB 甚至几个 GB 的编译临时文件夹。
这些文件**绝对不能上传到 GitHub**。项目里已经为你创建好了 `.gitignore` 文件，Git 会自动屏蔽这些构建文件，只上传纯代码与配置文件。

---

## 2. GitHub 上传四大步骤

### 步骤 1：本地初始化 Git 仓库与提交
打开终端，进入项目根目录：
```bash
cd /home/freebird/AntigraProj/Arm_Grabbing

# 1. 初始化本地 Git 仓库
git init

# 2. 将项目文件添加到暂存区（.gitignore 规则会自动生效）
git add .

# 3. 查看要提交的文件状态（确认没有 build/ 或 install/ 文件夹）
git status

# 4. 提交本地历史版本
git commit -m "feat: initial commit for SO-ARM101 ROS2 & MoveIt2 workspace"
```

---

### 步骤 2：在 GitHub 网站上新建仓库
1. 打开浏览器登录 [GitHub.com](https://github.com)。
2. 点击右上角的 **`+`** 按钮 ➔ 选择 **`New repository`**。
3. **Repository name** 填入：`Arm_Grabbing` (或 `so_arm101_hybrid_control`)。
4. **Description** 填入：`ROS 2 & MoveIt 2 Control System for SO-ARM101 Robot Arm (Model-Based to Learning-Based)`。
5. 选择 **Public**（公开）。
6. **重要**：下面“Initialize this repository with...”中的三个勾选框（Add a README, Add .gitignore, Choose a license）**全部留空不要勾选**（因为本地已经创建好了）。
7. 点击 **`Create repository`** 创建。

---

### 步骤 3：将本地仓库与 GitHub 远程仓库关联并推送
创建成功后，页面会给出仓库的地址（支持 HTTPS 或 SSH）：

#### 方式 A：使用 HTTPS 关联推送
```bash
cd /home/freebird/AntigraProj/Arm_Grabbing

# 将默认分支更名为 main
git branch -M main

# 关联远程 GitHub 仓库（请替换 your_username 为你的 GitHub 用户名）
git remote add origin https://github.com/your_username/Arm_Grabbing.git

# 推送代码到 GitHub
git push -u origin main
```
*提示：如果使用 HTTPS 提示需要输入密码，GitHub 现在需要使用 Personal Access Token (PAT)，或者使用 SSH key。*

#### 方式 B：使用 SSH 密钥推送（推荐，最省心）
若你之前配置过 GitHub SSH 密钥：
```bash
git remote add origin git@github.com:your_username/Arm_Grabbing.git
git push -u origin main
```

---

### 步骤 4：后续更新代码的标准流程
以后每次跟着教程写完新功能（比如完成 C++ 硬件驱动或视觉抓取）：
```bash
cd /home/freebird/AntigraProj/Arm_Grabbing

# 1. 检查修改了哪些文件
git status

# 2. 添加修改
git add .

# 3. 提交说明
git commit -m "feat: add STS3215 ros2_control C++ hardware interface"

# 4. 推送到 GitHub
git push
```
