from multiprocessing import Process, Value
import ctypes

web_distance_cm = Value(ctypes.c_double, 0.0)
web_angle_degrees = Value(ctypes.c_double, 90.0)
web_state = Value(ctypes.c_bool, True)

def sensor_loop(web_distance_cm, web_angle_degrees, web_state):
	import pigpio
	import time
	import statistics
	pi = pigpio.pi()

	# GPIO pins definitions
	servo_pwm = 18		# 12 on BOARD
	trig_ultrasonic = 23	# 16 on BOARD
	echo_ultrasonic = 24 	# 18 on BOARD
	green_led = 20		# 38 on BOARD
	red_led = 21		# 40 on BOARD
	button = 25		# 22 on BOARD

	# CONSTANTS
	TIMEOUT_ULTRASONIC = 28000	# us - for a max distance of 4 m it takes 24ms for a round trip
	DISTANCE_OFFSET = 9	# erorr of 9cm
	MAX_DISTANCE = 400 	# 400cm = 4m
	ITER_TIME_LIMIT = 650000

	# setting directions on pins
	pi.set_mode(servo_pwm, pigpio.OUTPUT)
	pi.set_mode(trig_ultrasonic, pigpio.OUTPUT)
	pi.set_mode(echo_ultrasonic, pigpio.INPUT)
	pi.set_mode(green_led, pigpio.OUTPUT)
	pi.set_mode(red_led, pigpio.OUTPUT)
	pi.set_mode(button, pigpio.INPUT)
	pi.set_pull_up_down(button,pigpio.PUD_UP)

	def button_pressed(gpio, level, tick):
		if level == 0:
			with web_state.get_lock():
				web_state.value = not web_state.value
	pi.set_glitch_filter(button,50000)
	cb = pi.callback(button,pigpio.FALLING_EDGE,button_pressed) 

	try:
		dir_value_min = 500
		dir_value_max = 2500
		step = -100
		step_degree = 180.0/abs((dir_value_max-dir_value_min)/step)
		threshold = 0
		i = dir_value_min
		while True:
			# stop the system if received command from interface
			if not web_state.value:
				pi.write(green_led,0)
				pi.write(red_led,1)
				time.sleep(0.1)
				continue
			else:
				pi.write(red_led,0)
				pi.write(green_led,1)
			total_time = pi.get_current_tick()
			samples = []
			pi.set_servo_pulsewidth(servo_pwm,i)
			for _ in range(7):
				distance_cm = 0
				duration_us = 0
				pi.write(trig_ultrasonic, 1)
				time.sleep(0.00001)
				pi.write(trig_ultrasonic, 0)
				start = pi.get_current_tick()

				while pi.read(echo_ultrasonic)==0:
					if pigpio.tickDiff(start, pi.get_current_tick()) > TIMEOUT_ULTRASONIC:
						distance_cm = 0.0
						print("ERROR: ECHO HIGH TIMEOUT; Object too close or missing")
						break
				else:
					start = pi.get_current_tick()
					while pi.read(echo_ultrasonic)==1:
						if pigpio.tickDiff(start, pi.get_current_tick()) > TIMEOUT_ULTRASONIC:
							distance_cm = 0
							print("ERROR: ECHO LOW TIMEOUT")
							break
					else:
						stop = pi.get_current_tick()
						duration_us = pigpio.tickDiff(start,stop)			# us
						distance_cm = (0.0343*duration_us)/2 - DISTANCE_OFFSET		# cm
				if distance_cm < MAX_DISTANCE:
					samples.append(distance_cm)
				wait_time = pi.get_current_tick()
				while True:
					if pigpio.tickDiff(wait_time,pi.get_current_tick()) >= 60000:
						break
			distance_cm = statistics.median(samples)
			with web_distance_cm.get_lock(), web_angle_degrees.get_lock():
				web_distance_cm.value = distance_cm
				web_angle_degrees.value = 0 + abs((i-dir_value_min)/step)*step_degree
			#print(*samples)
			while True:
				if pigpio.tickDiff(total_time,pi.get_current_tick()) >= ITER_TIME_LIMIT:
					break
			if i==dir_value_max or i==dir_value_min:
				step=-step
			i = i+step
	finally:
		cb.cancel()
		pi.set_servo_pulsewidth(servo_pwm,1500)
		time.sleep(1)
		pi.set_servo_pulsewidth(servo_pwm,0)
		pi.stop()

def run_web():
	from bottle import route, run, response
	import json
	@route("/")
	def index():
		response.content_type = "text/html"
		return """
<!DOCTYPE html>
<html>
<head>
	<title>Ultrasonic Mapping</title>
	<style>
		body { background:#111; color:#eee; text-align:center; }
		canvas { background:#000; border:1px solid #444; }
		#ui-container { display: flex; flex-direction: column; align-items: center; gap:12px; }
		#btn-state { width: 160px; height: 60px; font-size: 2.5rem; border-radius: 20px; }
		.btn-on { background: #d32f2f; }
		.btn-off { background: #388e3c; }
	</style>
</head>
<body>
<h2>Mapping system </h2>
<div id="ui-container">
	<canvas id="map" width="900" height="460"></canvas>
	<button class="btn-on" id="btn-state" width="200" height="120">STOP</button>
</div>
<script>
const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");

const MAX_DISTANCE = 400;	//cm
const cx = canvas.width / 2;
const cy = 400;
const R = 400;

const ANGLE_UNIT = 9	// degrees
const environment_map = new Array(180/ANGLE_UNIT).fill(null);

const LABEL_DIST_STEP = 100;	//cm

function clearCanvas() {
	ctx.clearRect(0,0,canvas.width,canvas.height)
}
function drawGrid() {
	ctx.strokeStyle = "#333";
	for (let r=0.125; r<=1; r+=0.125) {
		ctx.beginPath();
		ctx.arc(cx,cy,R*r,Math.PI,0);
		ctx.stroke();
	}
}
function drawDistanceLabels() {
	ctx.fillStyle="#aaa";
	ctx.font = "12px monospace";
	ctx.textAlign = "center";
	ctx.textBaseline = "top";

	for(let d=-MAX_DISTANCE; d<=MAX_DISTANCE; d+=LABEL_DIST_STEP) {
		const x = cx+(d/MAX_DISTANCE)*R;
		const y = cy+5;
		ctx.fillText(d.toString(),x,y);
	}
}
function drawRay(angle,distance) {
	const rad = angle * Math.PI / 180 - Math.PI;
	const dx = Math.cos(rad);
	const dy = Math.sin(rad);

	// full ray - green ( free space ), yellow ( error at measuring )
	ctx.strokeStyle = "green";
	if(distance == 0) ctx.strokeStyle = "yellow";
	ctx.beginPath();
	ctx.moveTo(cx,cy);
	ctx.lineTo(cx+R*dx,cy+R*dy);
	ctx.stroke();

	if ( distance > 0 && distance < MAX_DISTANCE) {
		const dPix = (distance/MAX_DISTANCE)*R;
		const ox = cx + dPix*dx;
		const oy = cy + dPix*dy;

		// interrupted line - red ( obstacle )
		ctx.strokeStyle = "red";
		ctx.beginPath();
		ctx.moveTo(ox,oy);
		ctx.lineTo(cx+R*dx,cy+R*dy);
		ctx.stroke();

		// obtacle
		ctx.fillStyle = "red"
		ctx.beginPath();
		ctx.arc(ox,oy,3,0,2*Math.PI);
		ctx.fill();
	}
}
const btn = document.getElementById("btn-state");
let current_state = true;

async function update() {
	try {
		const r = await fetch("/data");
		const d = await r.json();

		if (current_state !== d.state ) {
			current_state = d.state;
			btn.textContent = current_state ? "STOP" : "START";
			btn.className = current_state ? "btn-on" : "btn-off";
		}
		if ( current_state ) {
			const idx = Math.round(d.angle/ANGLE_UNIT);
			environment_map[idx] = d.distance;
			clearCanvas();
			drawGrid();
			drawDistanceLabels();
			for(let i=0;i<environment_map.length;i++) {
				if(environment_map[i] !== null) {
					drawRay(i*ANGLE_UNIT, environment_map[i]);
				}
			}
		}
	} catch(e) {  console.log("Fetch error", e); }
}
btn.onclick = async () => {
        const cmd = current_state ? "stop" : "start";
        const r = await fetch(`/control/${cmd}`);
        const d = await r.json();
};

drawGrid();
setInterval(update,600);
</script>
</body>
</html>
"""
	@route("/data")
	def data():
		return {
		"angle":web_angle_degrees.value,
		"distance": web_distance_cm.value,
		"state": web_state.value
		}

	@route("/control/<cmd>")
	def command(cmd):
		if cmd == "start":
			web_state.value = True
		elif cmd == "stop":
			web_state.value = False
		return { "state": web_state.value }

	run(host='0.0.0.0', port=5000, debug=False, reloader=False)

if __name__ == '__main__':
	p1 = Process(target=sensor_loop, args=(web_distance_cm,web_angle_degrees, web_state))
	#p2 = Process(target=run_web)

	#p1.daemon = True
	#p2.daemon = True

	p1.start()
	#p2.start()
	run_web()

	try:
		while p1.is_alive():
			p1.join(timeout=1.0)
	except KeyboardInterrupt:
		print("\nUser requested stop. Shutting down...")
