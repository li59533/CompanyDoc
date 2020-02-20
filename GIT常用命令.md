# 常用Git命令
## 推送远程服务器
1.git push [remote -name][branch -name]  
将本地库（跟踪）推送到远程库
2.git push origin --tags    
将本地所有tags推送到远程

## 分支
1.git branch [branch-name]  
创建分支    
2.git branch -a     
查看所有分支    
3.git checkout [branch-name]    
切换分支
4.git branch -d [branch-name]   
删除目标分支

## 合并 
1.git merge [branch-name]   
将当前库和目标库合并

## 其他
1.git status    
查看当前git库的文件状态

2.git commit --amend    
重新提交

3.git tag -a [tag name] -m "[message]" 附注标签

