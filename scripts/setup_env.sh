if [ -n "$ZSH_VERSION" ]; then
    _setup="setup.zsh"
elif [ -n "$BASH_VERSION" ]; then
    _setup="setup.bash"
else
    _setup="setup.sh"
fi

for _distro in jazzy humble iron; do
    if [ -f "/opt/ros/${_distro}/${_setup}" ]; then
        . "/opt/ros/${_distro}/${_setup}"
        break
    fi
done

if [ -f "install/${_setup}" ]; then
    . "install/${_setup}"
fi

unset _setup _distro
