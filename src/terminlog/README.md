# terminlog

ROS2 node that subscribe to `/rosout` topic and print the log message to tui application that base on textual package

## issue
- log message from the terminlog node itself print to console application
- launch file fail to open run application 
  - need to run in tmux or terminal session , beater to use `ros2 run` command


## Note

In ROS 2, log messages at `DEBUG` and UNSET levels do not appear in the `/rosout` topic by default.
This happens because the default log level for the /rosout topic is INFO.

You need to enable /set logging level

- code
- cli           # TBD
- parameter     # TBD

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().set_level(logging.DEBUG)  # Enable DEBUG logs
```


## Patch rule file
- deb output
- post inst script


```make
override_dh_install:
	dh_install
	mkdir -p debian/tmp/DEBIAN
	cp $(CURDIR)/my_debian/postinst debian/postinst
	chmod 755 debian/postinst

override_dh_builddeb:
	dh_builddeb --destdir=/home/user/workspaces/termilog_ws/deb
```

### Docker to test deb installation
```bash
docker run -it --rm --net host --user user --hostname test -v `pwd`/deb:/tmp/debs humble:dev /bin/bash
```