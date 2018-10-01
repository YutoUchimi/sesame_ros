# sesame_ros

ROS API for Sesame Smartlock made by CANDY HOUSE, Inc.


## API key

You have to get API key from CANDY HOUSE Dashboard: https://my.candyhouse.co/


## Usage

1. Start service server.
```
$ rosrun sesame_ros sesame_server.py _auth_token:=YOUR_AUTH_TOKEN
```

2. Call service.

    - To get Sesame status:
    ```
    $ rosservice call /sesame_server/get_status
    ```

    - To lock Sesame:
    ```
    $ rosservice call /sesame_server/lock
    ```

    - To unlock Sesame:
    ```
    $ rosservice call /sesame_server/unlock
    ```

    - To force the server to update Sesame status:
    ```
    $ rosservice call /sesame_server/force_sync
    ```
