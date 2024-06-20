# JLL  2023.4.7
# from Cython for absolute beginners: 30x faster code
# for 230406 Error 2: params_pyx.pyx

def count_primes(limit:int) -> int:
  """ Vanilla python that returns the number of primes between 0 and [limit] """
  count:int = 0
  for candidate_int in range(limit):
    if (candidate_int > 1):
      for factor in range(2, candidate_int):
        if candidate_int % factor == 0:
          break
      else:
        count += 1
  return count
