import motion as pi
import math 


def control_ip(v,w,d,l):
	
	#v_r=v+(l/2)*w
	v_r=(1/d)*(2*v+l*w)
	v_l=(1/d)*(2*v-l*w)
	#v_l=v-(l/2)*w
	
	if v_l>=0:
		if v_r>=0:
			pwm_l=math.floor((156/3.8)*v_l+91)
			pwm_r=math.floor((158/3.8)*v_r+97)
			pi.velocity(pwm_l,pwm_r,1)
			print("mode:",1)
		elif v_r<0:
			pwm_l=math.floor((156/3.8)*v_l+91)
			pwm_r=math.floor((158/3.8)*abs(v_r)+97)
			print(v_r)
			pi.velocity(pwm_l,pwm_r,2)
			print("mode:",2)

	elif v_l<0:
		if v_r>=0:
			pwm_l=math.floor((156/3.8)*abs(v_l)+91)
			pwm_r=math.floor((158/3.8)*v_r+97)
			pi.velocity(pwm_r,pwm_r,3)
			print("mode:",3)
		elif v_r<0:
			pwm_l=math.floor((156/3.8)*abs(v_l)+91)
			pwm_r=math.floor((158/3.8)*abs(v_r)+97)
			pi.velocity(pwm_l,pwm_r,4)
			print("mode:",4)


	
