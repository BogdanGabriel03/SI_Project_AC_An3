from multiprocessing import Process, Value
import ctypes

web_distance_cm = Value(ctypes.c_double, 0.0)
web_angle_degrees = Value(ctypes.c_double, 90.0)

def sensor_loop(web_distance_cm, web_angle_degrees):
	import pigpio
	import time
	import statistics
	pi = pigpio.pi()
	servo_pwm = 18		# GPIO pin for servo direction == 12 on BOARD
	trig_ultrasonic = 23	# GPIO pin of ultrasonic trigger == 16 on BOARD
	echo_ultrasonic = 24 	# GPIO pin of ultrasonic echo == 18 on BOARD
	TIMEOUT_ULTRASONIC = 28000	# us - for a max distance of 4 m it takes 24ms for a round trip
	DISTANCE_OFFSET = 9	# erorr or 9cm
	MAX_DISTANCE = 400 	# 400cm = 4m
	ITER_TIME_LIMIT = 550000

	# setting directions on pins
	pi.set_mode(servo_pwm, pigpio.OUTPUT)
	pi.set_mode(trig_ultrasonic, pigpio.OUTPUT)
	pi.set_mode(echo_ultrasonic, pigpio.INPUT)

	try:
		dir_value_min = 500
		dir_value_max = 2500
		step = -100
		step_degree = 180.0/abs((dir_value_max-dir_value_min)/step)
		threshold = 0
		i = dir_value_min
		while True:
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
				time.sleep(0.06)
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
	</style>
</head>
<body>
<h2>Mapping system </h2>
<canvas id="map" width="900" height="460"></canvas>

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
async function update() {
	const r = await fetch("/data");
	const d = await r.json();

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
drawGrid();
setInterval(update, 500);
</script>
</body>
</html>
"""
	@route("/data")
	def data():
		return {
		"angle":web_angle_degrees.value,
		"distance": web_distance_cm.value
		}

	run(host='0.0.0.0', port=5000, debug=False, reloader=False)

if __name__ == '__main__':
	p1 = Process(target=sensor_loop, args=(web_distance_cm,web_angle_degrees))
	p2 = Process(target=run_web)

	p1.daemon = True
	p2.daemon = True

	p1.start()
	p2.start()

	try:
		while p1.is_alive() and p2.is_alive():
			p1.join(timeout=1.0)
			p2.join(timeout=1.0)
	except KeyboardInterrupt:
		print("\nUser requested stop. Shutting down...")
