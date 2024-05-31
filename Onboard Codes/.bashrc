#!/bin/bash
# ModalAI default bashrc file

#VERSION 1.2

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# ubuntu ssh sets TERM to xterm-256color which screws with ROS
# change it to linux spec
export TERM=linux

if [ -d /home/root/.profile.d/ ]; then
  for i in /home/root/.profile.d/* ; do
    if [ -d $i ]; then
        for j in $i/* ; do
            if [ -f $j ]; then
                . $j
            fi
        done
    else
        . $i
    fi
  done
fi

# Put user-specific tweaks here or in a file in ~/.profile.d/

##########
#source ros2_ws/install/setup.bash
alias cb='colcon build && source install/setup.bash'
alias sis='source install/setup.bash'
source ros2_ws/install/setup.bash 
source docker_ws/install/setup.bash 
#export ROS_DOMAIN_ID=0
export XRCE_DOMAIN_ID_OVERRIDE=9
export ROS_DOMAIN_ID=9
