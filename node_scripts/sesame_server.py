#!/usr/bin/env python

import time

import numpy as np
import requests

import rospy
from sesame_ros.srv import Command
from sesame_ros.srv import CommandResponse
from sesame_ros.srv import Status
from sesame_ros.srv import StatusResponse


class SesameServer(object):

    def __init__(self):
        self.auth_token = rospy.get_param('~auth_token')
        self.device_id = rospy.get_param('~device_id', None)
        self.command_timeout = 60
        self.status_server = rospy.Service(
            '~get_status', Status, self.get_sesame_status)
        self.sync_server = rospy.Service(
            '~force_sync', Command, self.force_sync)
        self.lock_server = rospy.Service('~lock', Command, self.lock)
        self.unlock_server = rospy.Service('~unlock', Command, self.unlock)

    def _get_sesame(self):
        ret = requests.get(
            'https://api.candyhouse.co/public/sesames',
            headers={'Authorization': self.auth_token})
        if ret.status_code == 200:
            sesames = ret.json()
            if self.device_id is not None:
                for sesame_ in sesames:
                    if self.device_id == sesame_['device_id']:
                        self.sesame = sesame_
            else:
                rospy.logwarn(
                    '~device_id is not specified. The first will be used.')
                self.sesame = sesames[0]
        else:
            rospy.logerr('[{}] {}'.format(ret.status_code, ret.text))
            rospy.logerr('No sesames found.')
            self.sesame = {'nickname': None, 'serial': None, 'device_id': None}

    def _get_sesame_status(self):
        battery = None
        locked = None
        responsive = None
        if not hasattr(self, 'sesame'):
            self._get_sesame()
        if self.sesame['device_id'] is not None:
            ret = requests.get(
                'https://api.candyhouse.co/public/sesame/{}'.
                format(str(self.sesame['device_id'])),
                headers={'Authorization': self.auth_token})
            if ret.status_code == 200:
                battery = np.int32(ret.json()['battery'])
                locked = bool(ret.json()['locked'])
                responsive = bool(ret.json()['responsive'])
            else:
                rospy.logerr('[{}] {}'.format(ret.status_code, ret.text))
        return self.sesame['nickname'], self.sesame['serial'], \
            self.sesame['device_id'], battery, locked, responsive

    def get_sesame_status(self, req):
        return StatusResponse(*self._get_sesame_status())

    def _post_command(self, command):
        if not hasattr(self, 'sesame'):
            self._get_sesame()
        ret = requests.post(
            'https://api.candyhouse.co/public/sesame/{}'.
            format(str(self.sesame['device_id'])),
            headers={'Authorization': self.auth_token},
            json={'command': command})
        if ret.status_code == 200:
            task_id = ret.json()['task_id']
            return task_id
        else:
            rospy.logerr('[{}] {}'.format(ret.status_code, ret.text))
            return None

    def _get_task_status(self, task_id):
        status = None
        successful = None
        error = None
        ret = requests.get(
            'https://api.candyhouse.co/public/action-result?task_id={}'.
            format(task_id),
            headers={'Authorization': self.auth_token})
        if ret.status_code == 200:
            status = ret.json()['status']
            if status == 'terminated':
                successful = bool(ret.json()['successful'])
                error = str(ret.json().get('error'))
        else:
            rospy.logerr('[{}] {}'.format(ret.status_code, ret.text))
        return status, successful, error

    def _post_command_and_wait(self, command, timeout):
        time_limit = time.time() + timeout
        task_id = self._post_command(command)
        time.sleep(3.0)
        status, successful, error = self._get_task_status(task_id)
        while status != 'terminated':
            if time.time() > time_limit:
                rospy.logerr('Operation Timeout. command: {}'.format(command))
                # Wait for updating status
                time.sleep(5.0)
                status, successful, error = self._get_task_status(task_id)
                break
            elif status is None:
                rospy.logerr('Task status is None.')
                break
            else:  # status == 'processing'
                time.sleep(1.0)
                status, successful, error = self._get_task_status(task_id)
        return status, successful, error

    def force_sync(self, req):
        return CommandResponse(
            *self._post_command_and_wait('sync', self.command_timeout))

    def lock(self, req):
        return CommandResponse(
            *self._post_command_and_wait('lock', self.command_timeout))

    def unlock(self, req):
        return CommandResponse(
            *self._post_command_and_wait('unlock', self.command_timeout))


if __name__ == '__main__':
    rospy.init_node('sesame_server')
    app = SesameServer()
    rospy.spin()
