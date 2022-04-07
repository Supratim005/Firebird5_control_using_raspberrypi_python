import motion as pi


def control_ip(v,w,d,l):
	r=d/2  #redious
	v_r=(1/2*r)*(2*v-l*w)

	v_l=(1/2*r)*(2*v+l*w)

	pwm_l=(160/0.1)*v_l+91

	pwm_r=(158/0.1)*v_r+97

	if v>=0: 
		pi.velocity(pwm_l,pwm_r,1)
	elif v<0:
		pi.velocity(pwm_l,pwm_r,2)


	return 