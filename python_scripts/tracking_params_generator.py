import numpy as np

class IdentifierParamsGenerator:
	def get_params(self): pass

class AllTragetIdentifierParamsGenerator(IdentifierParamsGenerator):
	__type = "all_target"
	__residual = 0
	
	def __init__(self, residual):
		self.__residual = residual
	
	def get_params(self):
		params = { 
			'type': self.__type,
			'res': self.__residual,
		}
		
		return params

##########################################################

class InitializerParamsGenerator:
	def get_params(self): pass

class PointsInitializerParamsGenerator(InitializerParamsGenerator):
	__type = "by_points"
	__points = 0
	
	def __init__(self, points):
		self.__points = points
		
	def get_params(self):
		params = { 
			'type': self.__type ,
			'points': self.__points,
		}
		
		return params

##########################################################

class ExtrapolatorParamsGenerator:
	def get_params(self): pass

class LinearExtrapolatorParamsGenerator(ExtrapolatorParamsGenerator):
	__type = "linear_extrapolator"
	
	def get_params(self):
		params = { 'type': self.__type }
		
		return params


##########################################################

class FilterParamsGenerator:
	def get_params(self): pass

class AlphaBetaParamsGenerator(FilterParamsGenerator):
	__type = "alpha_beta"
	__alpha = 0
	__beta = 0
	
	def __init__(self, alpha, beta):
		self.__alpha = alpha
		self.__beta = beta
	
	def get_params(self):
		params = {
			'type': self.__type,
			'alpha': self.__alpha,
			'beta': self.__beta,
		}
		
		return params
 
class KalmanParamsGenerator(FilterParamsGenerator):
	__type = "static_kalman"
	__F = np.empty((2,2))
	__H = np.empty((1,2))
	__Q = np.empty((2,2))
	__R = np.empty((1,1))
	__P = np.empty((2,2))
	__Xk_ = np.empty((2,1))
	
	def __init__(self, F=None, H=None, Q=None, R=None, P=None, Xk_=None):
		self.__F = F
		self.__H = H
		self.__Q = Q
		self.__R = R
		self.__P = P
		self.__Xk_ = Xk_
	
	def set_type(self, type):
		self.__type = type
		
	def set_Q(self, Q):
		self.__Q = Q
	
	def set_R(self, R):
		self.__R = R
		
	def set_P(self, P):
		self.__P = P
	
	def __get_matrix_params(self, matrix):
		matrix_params = {
			'M': matrix.shape[0],
			'N': matrix.shape[1],
			'values': ' '.join(str(n) for n in np.nditer(matrix)),
		}
		return matrix_params
	
	def get_params(self):
		params = {
			'type': self.__type,
			'F': self.__get_matrix_params(self.__F),
			'H': self.__get_matrix_params(self.__H),
			'Q': self.__get_matrix_params(self.__Q),
			'R': self.__get_matrix_params(self.__R),
			'P': self.__get_matrix_params(self.__P),
			'Xk_': self.__get_matrix_params(self.__Xk_),
		}
		
		return params

class DynamicKalmanParamsGenerator(FilterParamsGenerator):
	__type = "dynamic_kalman"
	__velocity = 0
	__StaticKalmanGenerator = KalmanParamsGenerator()
	
	def __init__(self, F, H, Q, R, P, Xk_, velocity):
		KalmanGenerator = KalmanParamsGenerator(F, H, Q, R, P, Xk_)
		self.__StaticKalmanGenerator = KalmanGenerator
		self.__StaticKalmanGenerator.set_type(self.__type)
		self.__velocity = velocity
	
	def get_params(self):
		params = self.__StaticKalmanGenerator.get_params()
		params['velocity'] = self.__velocity
		return params

		
class UnscentedKalmanParamsGenerator(FilterParamsGenerator):
	__type = "unscented_kalman"
	__Q = np.empty((2,2))
	__R = np.empty((2,2))
	__P = np.empty((2,2))
	__Xk_ = np.empty((2,1))
	__velocity = 0
	__distance = 0
	__alpha = 0
	__beta = 0
	__kappa = 0
	
	def __init__(self, Q=None, R=None, P=None, Xk_=None, velocity=None, distance=None, alpha=None, beta=None, kappa=None):
		self.__Q = Q
		self.__R = R
		self.__P = P
		self.__Xk_ = Xk_
		self.__velocity = velocity
		self.__distance = distance
		self.__alpha = alpha
		self.__beta = beta
		self.__kappa = kappa
		
	
	def set_type(self, type):
		self.__type = type
		
	def set_Q(self, Q):
		self.__Q = Q
	
	def set_R(self, R):
		self.__R = R
		
	def set_P(self, P):
		self.__P = P
	
	def __get_matrix_params(self, matrix):
		matrix_params = {
			'M': matrix.shape[0],
			'N': matrix.shape[1],
			'values': ' '.join(str(n) for n in np.nditer(matrix)),
		}
		return matrix_params
	
	def get_params(self):
		params = {
			'type': self.__type,
			'Q': self.__get_matrix_params(self.__Q),
			'R': self.__get_matrix_params(self.__R),
			'P': self.__get_matrix_params(self.__P),
			'Xk_': self.__get_matrix_params(self.__Xk_),
			'alpha': self.__alpha,
			'beta': self.__beta,
			'kappa': self.__kappa,
			'distance': self.__distance,
			'velocity': self.__velocity,
		}
		
		return params
	

class UnscentedBankParamsGenerator(FilterParamsGenerator):
	__type = "unscented_bank"
	__scale_koef = 0
	__UnscentedGenerator = UnscentedKalmanParamsGenerator()
	
	def __init__(self, Q, R, P, Xk, velocity, kf, alpha=None, beta=None, kappa=None):
		UnscentedGenerator = UnscentedKalmanParamsGenerator(Q, R, P, Xk, velocity, None, alpha, beta, kappa)
		self.__UnscentedGenerator = UnscentedGenerator
		self.__UnscentedGenerator.set_type(self.__type)
		self.__scale_koef = kf
	
	def get_params(self):
		params = self.__UnscentedGenerator.get_params()
		params['coef'] = self.__scale_koef
		return params


##########################################################

class ParamsGenerator:
	__IdentifierGenerator   = IdentifierParamsGenerator()
	__FilterGenerator       = FilterParamsGenerator()
	__InitializerGenerator  = InitializerParamsGenerator()
	__ExtrapolatorGenerator = ExtrapolatorParamsGenerator()
	
	def __init__(self, IdentifierGenerator, 
                     FilterGenerator, 
										 InitializerGenerator,
										 ExtrapolatorGenerator):
										 
		self.__IdentifierGenerator   = IdentifierGenerator
		self.__FilterGenerator       = FilterGenerator
		self.__InitializerGenerator  = InitializerGenerator
		self.__ExtrapolatorGenerator = ExtrapolatorGenerator
		
	def set_filter_generator(self, FilterGenerator):
		self.__FilterGenerator = FilterGenerator
		
		
	def get_params(self):
		params = {
			'tracking_object': {
				'Filter' : self.__FilterGenerator.get_params(),
				'Initializer' : self.__InitializerGenerator.get_params(),
				'Extrapolator' : self.__ExtrapolatorGenerator.get_params(),
			},
			'identifier': self.__IdentifierGenerator.get_params(),
		}
		return params
		
#################### For testing ########################

def test_module():

	F = np.empty((2,2))
	H = np.empty((1,2))
	Q = np.empty((2,2))
	R = np.empty((1,1))
	P = np.empty((2,2))
	Xk_ = np.empty((2,1))
	
	print(F)
	
	IdentifierGenerator = AllTragetIdentifierParamsGenerator(10000)
	FilterGenerator = KalmanParamsGenerator(F, H, Q, R, P, Xk_)
	InitializerGenerator = PointsInitializerParamsGenerator(2)
	ExtrapolatorGenerator = LinearExtrapolatorParamsGenerator()
	
	Generator = ParamsGenerator(IdentifierGenerator, FilterGenerator, InitializerGenerator, ExtrapolatorGenerator)
	print(Generator.get_params())
