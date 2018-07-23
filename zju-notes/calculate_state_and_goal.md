# calculate_state_and_goal

​	这个函数在整个工程中属于核心函数，就是判断当前运动状态的函数

路径：/home/wangxuexi/sailing-robot/src/sailing_robot/src/sailing_robot/heading_planning_laylines.py

猜想：这个文件是对heading_planning.py的改进，增加了判断判断三中的区域判断

# 函数详解：

​	因为函数中设计到很多次的判断，所以我记录了详细解释了总体描述，如果想看总体描述可以直接移步到文本的最后一部分，”函数总结描述“

------

## 变量含义：

​	dwp：distance to wp

​	hwp：heading to wp

​	waypoint_id：当前目标点的序号(也就是table里面的名字)

​	goal_angle：这个是一个设定的角度，为常量，用来判断迎风状态是否能前行的（这里设为+-50）

​	on_port_tack：判断是左舷还是右舷

​	wp_wind_angle：目标方向相对风向的角度

### 判断一

```python
if self.sailing_state != 'normal':
    # A tack/jibe is in progress
    if self.sailing_state == 'switch_to_port_tack':
        goal_angle = self.nav.beating_angle
        continue_tack = boat_wind_angle < goal_angle or boat_wind_angle > 120
    else:  # 'switch_to_stbd_tack'
        goal_angle = -self.nav.beating_angle
        continue_tack = boat_wind_angle > goal_angle or boat_wind_angle < -120

    if continue_tack:
        self.debug_pub('dbg_goal_wind_angle', goal_angle)
        return self.sailing_state, self.nav.wind_angle_to_heading(goal_angle)
    else:
        # Tack completed
        self.log('info', 'Finished tack (%s)', self.sailing_state)
        self.tack_voting.reset(boat_wind_angle > 0)
        self.sailing_state = 'normal'
```

​	先判读sailing_state不是‘normal’的状态，可以看到这个情况下唯一会退出函数的的情况就是，需要继续tack会有返回值。

​	经过串口输出可以发现，goal_angle的值：

​		sailing_state == 'switch_to_port_tack'，为50

​		sailing_state == 'switch_to_stbd_tack'，为-50	   两者都是定值，初步判定为直接设置的参数。

- 第一个判断是两个的夹角关系来判断是否还需要继续保持tack这个状态，其实就是为了保证转弯结束后是一个能够迎风吃风前进的状态，否则不再tack会让船因为迎风角太小而速度极具下降。可以看到用来判断continue_tack状态的有两种条件，满足一种就可以，另一种条件就是走8字型情况下，如果角度大于120°就结束这个状态，否则会顺风走远（虽然仿真结果看起来有点奇怪）
- 第二个判断是看其是否需要继续来做出想要的动作，这里两个函数我们分别下去看

```python
if continue_tack:
	self.debug_pub('dbg_goal_wind_angle', goal_angle)
	return self.sailing_state, self.nav.wind_angle_to_heading(goal_angle)
```

其中用到的函数为

```python
def wind_angle_to_heading(self, wind_angle):
    """Convert angle relative to the wind (+-180) to a compass heading (0-360).
    """
    return angleSum(self.absolute_wind_direction(), wind_angle)
```

​	注释的意思为 将相对于风的角度（+ -180）转换为罗盘标题（0-360），虽然感觉有些奇怪。



下面对应上面的if

```python
else:
    # Tack completed
    self.log('info', 'Finished tack (%s)', self.sailing_state)
    self.tack_voting.reset(boat_wind_angle > 0)
    self.sailing_state = 'normal'
```

其中用到的函数为

```python
def reset(self, current_tack):
    """Set all votes one way.

    :param int current_tack: 0 on startboard tack, 1 on port tack.
    """
    if current_tack:
        self.votes.extend([1] * self.nsamples)
        self.votes_sum = self.nsamples
    else:
        self.votes.clear()
        self.votes_sum = 0
```

​		current_tack参数是由boat_wind_angle来判断，也可以简单的认为是判断左舷右舷



------

### 判断二

```python
if (wp_wind_angle % 360) > 90 and (wp_wind_angle % 360) < 270:
    goal_wind_angle = wp_wind_angle
    self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
    state = 'normal'
    return state, self.nav.wind_angle_to_heading(goal_wind_angle)
```

​	该状态可以理解为顺风状态，当前设定前进方向 goal_wind_angle就是目标点所在方向wp_wind_angle。并继续了这个函数中第二种情况的返回值。



------

### 判断三

```python
tack_now = False
if wp_wind_angle * boat_wind_angle > 0:
    # These two have the same sign, so we're on the better tack already
    self.tack_voting.vote(on_port_tack)
elif self.nav.position_xy.within(self.lay_triangle()):
    # We're between the laylines; stick to our current tack for now
    self.tack_voting.vote(on_port_tack)
else:
    tack_now = True
    self.tack_voting.vote(not on_port_tack)
```

这个判断中比较重要的函数如下

```python
    def lay_triangle(self):
        """Calculate the lay lines for the current waypoint.
        
        This returns a shapely Polygon with the two lines extended to
        LAYLINE_EXTENT (10km).
        """
        downwind = angleSum(self.nav.absolute_wind_direction(), 180)
        x0, y0 = self.waypoint_xy.x, self.waypoint_xy.y
        l1 = math.radians(angleSum(downwind, -self.nav.beating_angle))
        x1 = x0 + (LAYLINE_EXTENT * math.sin(l1))
        y1 = y0 + (LAYLINE_EXTENT * math.cos(l1))
        l2 = math.radians(angleSum(downwind, self.nav.beating_angle))
        x2 = x0 + (LAYLINE_EXTENT * math.sin(l2))
        y2 = y0 + (LAYLINE_EXTENT * math.cos(l2))
        return Polygon([(x0, y0), (x1, y1), (x2, y2)])
```

这里需要知道含义的变量有

LAYLINE_EXTENT：延伸范围长度（这里设定为10000m表示无穷大）

absolute_wind_direction：绝对风向

beating_angle：设定的迎风前进的最小角度



判断选项

1. 风向平行法：就是如果两个角度同号，则说明是处于一个不错的状态
2. 区域判断法：不用在意坐标计算的方法，就是一个三角区域范围，看是不是超过了这个范围
3. 如果都不满足则tack_now变成True



现在看懂了判断的三种条件的含义，我们再来看看内部的投票函数的意思：

```python
    def vote(self, value):
        """Push one vote.
        0 votes for starboard tack, 1 for port.
        """
        if len(self.votes) >= self.nsamples:
            self.votes_sum -= self.votes.popleft()
        self.votes.append(value)
        self.votes_sum += value
```

变量含义：

nsamples：设定的定值 50

​	每次投票都会奖当前的左舷还是右舷的状态记录的长度为50的数组的最后一位，如果数组超过50长度则讲最前面的弹出，然后sum是表示当前数组中所有元素总和。



------

### 判断四

```python
        if dwp < self.tack_voting_radius:
            # Close to the waypoint, use tack voting so we're not constantly
            # tacking.
            tack_now = self.tack_voting.tack_now(on_port_tack)
```

​	注释的意思是，当船接近wp的时候，我们就使用vote来决定是否需要tack，避免非常频繁的tack的出现（非常频繁的tack坏处很多）

​	下面来看判断中使用到的函数

```python
    def tack_now(self, current_tack):
        """Get the current result - True to tack now.
        :param int current_tack: 0 if on starboard tack now, 1 if on port tack
        """
        if current_tack:
            # Port: tack to starboard?
            return self.votes_sum < (self.nsamples - self.threshold)
        else:
            # Starboard: tack to port?
            return self.votes_sum > self.threshold
```

​	用votes_sum的值和当前的左舷右舷状态来判断是否需要tack。

​	也就是说可以结合上面一个判断，这里有两种方式判断是否需要tack，当离wp较远的时候，也就是距离大于设定的voting_radius时，我们用上一个判断的判据，如果小于这个距离则用下面的判据。



------

### 判断五

```python
        if tack_now:
            # Ready about!
            if on_port_tack:
                state = 'switch_to_stbd_tack'
                goal_wind_angle = -self.nav.beating_angle
            else:
                state = 'switch_to_port_tack'
                goal_wind_angle = self.nav.beating_angle
            self.sailing_state = state
            self.log('info', 'Starting tack/jibe (%s)', state)
        else:
            # Stay on our current tack
            if on_port_tack:
                goal_wind_angle = max(wp_wind_angle, self.nav.beating_angle)
            else:
                goal_wind_angle = min(wp_wind_angle, -self.nav.beating_angle)
            state = 'normal'
```

​	这个判断已经是最后一个判断，用来给我们的state来赋值等操作



------

```python
        self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
        return state, self.nav.wind_angle_to_heading(goal_wind_angle)
```

​	最后就是赋值和return的操作了，之前没有return的都将在这里return了



------

------

------

# 函数总体描述

​	经过上面的一个个判断和调用函数的分析，现在总结一下这个大函数的作用



### 判断一

```python
if self.sailing_state != 'normal':
```

​	小判断一：

​		用当前的boat_wind_angle是否是无效迎风角来判断是否要继续tack

​		用来继续和下面通信的参数为continue_tack

​	小判断二（continue_tack）：

​		True ：直接**`return`**当前记录的状态（switch）和运动目标角度，为beating_angle（定值）

​		False：说明完成了tack运动，初始化投票界面，初始化的时候传入参数为（boat_wind_angle > 0），然后讲当前状态设置为‘normal’



再多说一句，这个大判断如果结束后依旧没有退出函数，说明经过了小判断一和小判断二，最后出来的时候当前状态已经成为了‘normal’，说明后面判断的时候状态均为‘normal’



### 判断二

```python
if (wp_wind_angle % 360) > 90 and (wp_wind_angle % 360) < 270:
```

​	当前状态为顺风状态的话，直接**return**当前状态（normal）和运动目标角度（wp_wind_angle），这里返回的运动目标角度不同于上面。



### 判断三

每次进入前都将先初始化 tack_now = False

情况一和情况二

```python
if wp_wind_angle * boat_wind_angle > 0:

 elif self.nav.position_xy.within(self.lay_triangle()):
```

​	情况一：如果两角同号，简而言之就是不是远离目标的方向

​	情况二：如果在我们用beating_angle计算出来的三角区域内

​	满足上述两个条件都奖对当前状态进行投票，也就是如果是左舷状态就投左舷（1），如果是右舷状态就投右舷（0），就是投保持当前状态的票



情况三

```python
else:
```

​	这时候就是需要进行tack操作了，修改参数tack_now = True

​	但是也进行投票，对当前状态投反对票



### 判断四

```python
if dwp < self.tack_voting_radius:
```

​	前面的判断三可以说是用两种情况来判断是否需要进行tack操作，而这一步是为了避免在船离目标点很近的时候出现频繁tack的情况（因为可能会失速严重）。而这个地方是这种要用到投票出来的结果的地方，如果投票结果需要tack则进行tack操作，关于如何确定是否需要tack，可以看上面详解函数部分。

​	结合上面的判断三说明一下这个投票机制（距离小于voting_radius时候的）：

​	因为如果不是距离小于这个半径也不会进入判断四。当我们在判断三环节发现前两个情况都不满足的时候，就开始对当前情况投反对票，然后进入判断四。如果反对票没有达到阈值，即使在判断三的else我们已经讲tack_now赋值为True，这里还是会重新赋值为False。反之达到阈值就会给tack_now赋值为True。



### 判断五

情况一：

```python
if tack_now:
```

​	这个情况比较简单，对state进行赋值，即开始switch，在switch的过程中goal_wind_angle赋值为我们之前的一个常量，beating_angle



情况二：

```python
else:
```

​	state赋值为‘normal’，而goal_wind_angle也会赋值，但是这个原理我没有看懂



最后进行函数的return	，返回state和goal_wind_angle

------

------

------

# 简要总结

1. 如果正在switch，则我们判断是否需要继续，需要继续则直接返回
2. 再判断是否是顺风，如果顺风状态则直接返回
3. 用两种方式判断是否需要进行tack操作，中间包含了投票操作
4. 如果距离wp很近，则开始启用投票结果来看是否tack，而上面的判断3只是用于投票而不是判断tack
5. 最后就是一个总结性的判断，对state根据当前情况赋值，对goal_wind_angle也根据情况赋值