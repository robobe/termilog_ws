source ~/.bashrc
source /opt/ros/humble/setup.bash
source install/setup.bash

function parse_git_dirty {
  [[ $(git status --porcelain 2> /dev/null) ]] && echo "*"
}
function parse_git_branch {
  git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e "s/* \(.*\)/ (\1$(parse_git_dirty))/"
}

export PS1="\n\H \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] 🐢 $ "