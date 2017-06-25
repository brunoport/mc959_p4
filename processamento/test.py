from DetetorDeProduto import DetetorDeProduto

# YELLOW = 0
	# GREEN =  1
	# CLEAR BLUE = 2
	# RED = 4
	# DARK BLUE = 6
	# PURPLE = 7
dp = DetetorDeProduto()
print "yellow : "
print dp.detectColor(0)
print "green : "
print dp.detectColor(1)
print "clear blue : "
print dp.detectColor(2)
print "red : "
print dp.detectColor(4)
print "dark blue : "
print dp.detectColor(6)
print "purple : "
print dp.detectColor(7)