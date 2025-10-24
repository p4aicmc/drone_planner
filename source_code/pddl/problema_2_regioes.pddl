( define ( problem problem_1 )
( :domain harpia )
( :objects
	region_1 region_2 - region
	base_1 - base
)
( :init
	( picture_goal region_1 )
	( picture_goal region_2 )
	( at base_1 )
	( = ( distance base_1 base_1 ) 0.0000000000 )
	( = ( distance base_1 region_1 ) 126.2738489105 )
	( = ( distance base_1 region_2 ) 94.0538024899 )
	( = ( distance region_1 base_1 ) 126.2738489105 )
	( = ( distance region_1 region_1 ) 0.0000000000 )
	( = ( distance region_1 region_2 ) 129.3246029227 )
	( = ( distance region_2 base_1 ) 94.0538024899 )
	( = ( distance region_2 region_1 ) 129.3246029227 )
	( = ( distance region_2 region_2 ) 0.0000000000 )
	( = ( battery_capacity ) 100.0000000000 )
	( = ( discharge_rate_battery ) 0.1000000000 )
	( = ( velocity ) 7.0000000000 )
	( = ( input_capacity ) 1.0000000000 )
	( = ( battery_amount ) 100.0000000000 )
	( = ( input_amount ) 1.0000000000 )
	( = ( mission_length ) 0.0000000000 )
)
( :goal
	( and
		( taken_image region_1 )
		( taken_image region_2 )
		( at base_1 )
	))
)
