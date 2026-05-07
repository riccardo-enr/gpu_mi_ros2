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

# GZ_SIM_RESOURCE_PATH: resolve from this script's location so it works
# regardless of $PWD when the pixi environment is activated.
if [ -n "$ZSH_VERSION" ]; then
    _SCRIPT_DIR="$(cd "$(dirname "${(%):-%x}")" && pwd)"
else
    _SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fi
_GZ_MODELS="${_SCRIPT_DIR}/../external/PX4-gazebo-models/models"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}${_GZ_MODELS}"
unset _SCRIPT_DIR _GZ_MODELS
