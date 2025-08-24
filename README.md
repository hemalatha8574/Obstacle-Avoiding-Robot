
---

# Obstacle Avoidance Robot

## 🎯 Objective  
To design a robot that moves forward and avoids obstacles using an ultrasonic sensor.

---

## 🛠 Components Required  
- Arduino UNO  
- Ultrasonic Sensor HC-SR04  
- Motor Driver L298N  
- 2 DC Motors + Wheels  
- Robot Chassis & Battery  

---

## 🔌 Circuit Diagram  


HC-SR04 TRIG ---> Arduino D9
HC-SR04 ECHO ---> Arduino D10
Motor Driver IN1, IN2 ---> Arduino D5, D6
Motor Driver IN3, IN4 ---> Arduino D7, D8


---

## ⚙️ Working Principle  
- Ultrasonic sensor measures distance.  
- If obstacle < 20 cm → robot turns.  
- If clear → robot moves forward.  

---

## 🌍 Applications  
- Autonomous navigation robots.  
- Delivery bots.  
- Factory automation.  

---
