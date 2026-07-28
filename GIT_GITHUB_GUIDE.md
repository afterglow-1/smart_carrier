# Smart Carrier GitHub 协作指南

本指南记录本项目的 Git 和 GitHub 使用方式，适用于个人开发、多人协作、代码回退和发布版本管理。

## 当前状态

- GitHub 仓库：`afterglow-1/smart_carrier`
- `main`：稳定版本分支。已合并项目源码后，仓库主页默认查看该分支即可看到项目目录。
- `develop`：日常集成分支。
- 项目源码已作为普通文件纳入主仓库，`car_code` 下的内容不再是 Git 子模块。
- 根目录 `.gitignore` 会排除 PlatformIO 和常见编译产物。

## 查看 GitHub 中的项目代码

在 GitHub 仓库的 **Code** 页面，点击文件列表左上方的分支下拉框：

1. 选择 `main` 查看稳定版本。
2. 选择 `develop` 查看尚未发布的集成代码。
3. 视觉相关代码位于 `maixcam/`，例如 `maixcam/main.py` 和 `maixcam/ColorRecognize_optimized.py`。

如果仓库主页只显示 `README.md`，通常是因为正在查看旧的 `main` 提交或尚未切换到包含源码的分支。

## 分支规则

| 分支 | 用途 | 合并方式 |
| --- | --- | --- |
| `main` | 稳定、可交付版本 | 通过 Pull Request 从 `develop` 合并 |
| `develop` | 多个功能的集成分支 | 通过 Pull Request 从功能分支合并 |
| `feature/功能名` | 一项功能或一个开发者的工作分支 | 开发完成后合并到 `develop` |
| `hotfix/问题名` | 紧急修复稳定版本 | 从 `main` 创建，修复后合并回 `main`，再同步到 `develop` |

不要让多人直接向 `main` 推送，也不要在共享的 `main` 或 `develop` 使用强制推送。

## 第一次下载项目

```powershell
git clone https://github.com/afterglow-1/smart_carrier.git
Set-Location smart_carrier

git config user.name "你的 GitHub 用户名"
git config user.email "你的 GitHub 提交邮箱"
```

查看分支和当前状态：

```powershell
git branch -a
git status
```

## 日常开发与提交

每次开始新功能前，先更新 `develop`，再创建自己的功能分支：

```powershell
git switch develop
git pull --ff-only origin develop
git switch -c feature/vision-improvement
```

修改和测试完成后：

```powershell
git status
git diff
git add -A
git diff --cached --stat
git commit -m "feat: improve vision recognition"
git push -u origin feature/vision-improvement
```

随后在 GitHub 创建 Pull Request：

```text
feature/vision-improvement -> develop
```

经过审查和测试后再合并。准备发布稳定版本时，再创建：

```text
develop -> main
```

## 更新本地代码

在自己的功能分支开发前，先确认本地没有未提交改动。更新 `develop`：

```powershell
git switch develop
git pull --ff-only origin develop
```

将 `develop` 合并到 `main` 后，为保持两者同步，可以更新 `develop`：

```powershell
git switch develop
git pull --ff-only origin develop
git fetch origin
git merge origin/main
git push origin develop
```

## 安全回退代码

Git 的每个提交都是一个可回到的版本。提交前先使用 `git status` 和 `git diff` 检查改动。

### 放弃尚未提交的单个文件修改

```powershell
git restore maixcam/main.py
```

### 查看提交历史

```powershell
git log --oneline --decorate -20
```

### 撤销已经推送的提交

使用 `revert` 会新增一个反向提交，不会破坏其他成员的历史：

```powershell
git revert 提交哈希
git push origin develop
```

例如：

```powershell
git revert abc1234
git push origin develop
```

### 从旧提交恢复一个文件

```powershell
git restore --source 提交哈希 -- maixcam/main.py
git add maixcam/main.py
git commit -m "fix: restore previous vision implementation"
git push origin develop
```

不要在已共享的分支上使用以下命令，它们会重写历史并可能影响其他成员：

```powershell
git reset --hard
git push --force
```

## 标记稳定版本

每次确认 `main` 可用后，创建版本标签，方便以后准确回退或下载：

```powershell
git switch main
git pull --ff-only origin main
git tag -a v0.1.0 -m "first stable version"
git push origin v0.1.0
```

之后可以使用 `v0.1.0` 在 GitHub 的 Releases 或 Tags 页面定位这次稳定版本。

## GitHub 多人权限设置

仓库所有者应在 GitHub 中完成以下设置：

1. 进入 `Settings -> Collaborators`，邀请需要写权限的成员。
2. 进入 `Settings -> Branches -> Add branch protection rule`。
3. 对 `main` 设置保护：勾选 **Require a pull request before merging**、**Require approvals**，并禁止 force push。
4. 建议对 `develop` 也设置“必须通过 Pull Request 合并”。
5. 外部贡献者不必拥有写权限，可以 Fork 仓库后向本仓库提交 Pull Request。

不要共享 GitHub 密码或 Personal Access Token。

## GitHub 推送失败但网页可访问

浏览器能打开 GitHub，而 PowerShell 无法推送，常见原因是浏览器使用了代理扩展或系统代理，Git 没有走同一条网络路径。

优先尝试在代理软件中开启 TUN 模式，然后测试：

```powershell
Test-NetConnection github.com -Port 443
git ls-remote --heads origin develop
git push origin develop
```

当 `TcpTestSucceeded : True` 时，GitHub HTTPS 连接正常。

也可以为当前项目显式配置代理。将 `7890` 替换为代理软件实际的 HTTP 端口：

```powershell
git config --local http.proxy "http://127.0.0.1:7890"
git config --local https.proxy "http://127.0.0.1:7890"
git push origin develop
```

不再需要代理时移除该项目的配置：

```powershell
git config --local --unset http.proxy
git config --local --unset https.proxy
```

## 常用检查命令

```powershell
git status
git branch -vv
git remote -v
git log --oneline --decorate -10
git diff
git diff --cached
```

