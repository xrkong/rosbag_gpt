#!/bin/bash

# copy regular rosbag from main PC
REMOTE_USER=nuway1
REMOTE_HOST=192.168.5.30
REMOTE_PATH=/home/nuway1/incidentRecord
ROSBAG_NAME=rosbag2_2023_11_28*
LOCAL_PATH=./nov-28/inc

mkdir -p ${LOCAL_PATH}
#scp -r nuway1@192.168.5.30:/home/nuway1/incidentRecord/${ROSBAG_NAME} ./inc #/${ROSBAG_NAME}

ssh "${REMOTE_USER}@${REMOTE_HOST}" "ls -d ${REMOTE_PATH}/${ROSBAG_NAME}" | while read remote_dir; do
    # Copy each directory from remote to local
    scp -r "${REMOTE_USER}@${REMOTE_HOST}:${remote_dir}" "$LOCAL_PATH"
    
    # Optional: Set permissions for each copied directory
    local_dir_name=$(basename "$remote_dir")
    chmod -R 755 "${LOCAL_PATH}/${local_dir_name}"
done

# copy incident rosbag from main PC
REMOTE_PATH=/home/nuway1/snapshotTest
LOCAL_PATH=./nov-28/snap
mkdir -p ${LOCAL_PATH}
ssh "${REMOTE_USER}@${REMOTE_HOST}" "ls -d ${REMOTE_PATH}/${ROSBAG_NAME}" | while read remote_dir; do    
    # Copy each directory from remote to local
    scp -r "${REMOTE_USER}@${REMOTE_HOST}:${remote_dir}" "$LOCAL_PATH"
    
    # Optional: Set permissions for each copied directory
    local_dir_name=$(basename "$remote_dir")
    chmod -R 755 "${LOCAL_PATH}/${local_dir_name}"
done

# copy images rosbag from Orin
REMOTE_USER=orin
REMOTE_HOST=192.168.5.29
REMOTE_PATH=/home/orin/kingston/
LOCAL_PATH=./nov-28/orin
mkdir -p ${LOCAL_PATH}
ssh "${REMOTE_USER}@${REMOTE_HOST}" "ls -d ${REMOTE_PATH}/${ROSBAG_NAME}" | while read remote_dir; do
    # Copy each directory from remote to local
    scp -r "${REMOTE_USER}@${REMOTE_HOST}:${remote_dir}" "$LOCAL_PATH"
    
    # Optional: Set permissions for each copied directory
    local_dir_name=$(basename "$remote_dir")
    chmod -R 755 "${LOCAL_PATH}/${local_dir_name}"
done
