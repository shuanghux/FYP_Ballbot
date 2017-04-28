Exception "java.lang.ClassNotFoundException: com/intellij/codeInsight/editorActions/FoldingData"while constructing DataFlavor for: application/x-java-jvm-local-objectref; class=com.intellij.codeInsight.editorActions.FoldingData
Exception "java.lang.ClassNotFoundException: com/intellij/codeInsight/editorActions/FoldingData"while constructing DataFlavor for: application/x-java-jvm-local-objectref; class=com.intellij.codeInsight.editorActions.FoldingData
KalmanGain
KalmanGain
KalmanGain
Error using mvnpdf (line 129)
SIGMA must be a square, symmetric, positive definite matrix.

Error in KalmanGain (line 66)
        multiplier = mvnpdf(yt,mu,sig);
 
KalmanGain
mvnpdf(yt,mu,sig)

ans =

    0.0962

temp =0;
dbcont
66          multiplier = mvnpdf(yt,mu,sig);
dbcont
Error using mvnpdf (line 129)
SIGMA must be a square, symmetric, positive definite matrix.

Error in KalmanGain (line 66)
        multiplier = mvnpdf(yt,mu,sig);
 
KalmanGain
Error using mvnpdf (line 129)
SIGMA must be a square, symmetric, positive definite matrix.

Error in KalmanGain (line 66)
        multiplier = mvnpdf(yt,mu,sig);
 
Sigma

Sigma =

    0.1215    0.1212
    0.1212    0.1209

eig(Sigma)

ans =

    0.2424
    0.0001

[v d] = eig(Sigma)

v =

    0.7081   -0.7062
    0.7062    0.7081


d =

    0.2424         0
         0    0.0001

Sigma(1,2)-Sigma(2,1)

ans =

  -1.3878e-16

dbquit
R = [1 0;0 2]

R =

     1     0
     0     2

chol(R)

ans =

    1.0000         0
         0    1.4142

sqrtm(R)

ans =

    1.0000         0
         0    1.4142

R = [1 1e-16;0 2]

R =

    1.0000    0.0000
         0    2.0000

sqrtm(R)

ans =

    1.0000    0.0000
         0    1.4142

chol(R)

ans =

    1.0000    0.0000
         0    1.4142

R = [1 1e-10;0 2]

R =

    1.0000    0.0000
         0    2.0000

chol(R)

ans =

    1.0000    0.0000
         0    1.4142

R = [1 1e-5;0 2]

R =

    1.0000    0.0000
         0    2.0000

chol(R)

ans =

    1.0000    0.0000
         0    1.4142

KalmanGain
Error using mvnpdf (line 129)
SIGMA must be a square, symmetric, positive definite matrix.

Error in KalmanGain (line 66)
        multiplier = mvnpdf(yt,mu,sig);
 
help cholcov
 cholcov  Cholesky-like decomposition for covariance matrix.
    T = cholcov(SIGMA) computes T such that SIGMA = T'*T.  SIGMA must be
    square, symmetric, and positive semi-definite.  If SIGMA is positive
    definite, then T is the square, upper triangular Cholesky factor.
 
    If SIGMA is not positive definite, T is computed from an eigenvalue
    decomposition of SIGMA.  T is not necessarily triangular or square in
    this case.  Any eigenvectors whose corresponding eigenvalue is close to
    zero (within a small tolerance) are omitted.  If any remaining
    eigenvalues are negative, T is empty.
 
    [T,P] = cholcov(SIGMA) returns the number of negative eigenvalues of
    SIGMA, and T is empty if P>0.  If P==0, SIGMA is positive semi-definite.
 
    If SIGMA is not square and symmetric, P is NaN and T is empty.
 
    [T,P] = cholcov(SIGMA,0) returns P==0 if SIGMA is positive definite, and
    T is the Cholesky factor.  If SIGMA is not positive definite, P is a
    positive integer and T is empty.  [...] = cholcov(SIGMA,1) is equivalent
    to [...] = cholcov(SIGMA).
 
    Example:
    Factor a rank-deficient covariance matrix C.
        C = [2 1 1 2;1 2 1 2;1 1 2 2;2 2 2 3]
        T = cholcov(C)
        C2 = T'*T
    Generate data with this covariance (aside from random variation).
        C3 = cov(randn(10000,3)*T)
 
    See also chol.

    Reference page for cholcov

dbquit
cholcov(R)

ans =

     []

[~,err] = cholcov(R)

err =

   NaN

R

R =

    1.0000    0.0000
         0    2.0000

err~=0

ans =

  logical

   1

R = [1 0;0 2]

R =

     1     0
     0     2

[~,err] = cholcov(R)

err =

     0

R = [1 1e-16;0 2]

R =

    1.0000    0.0000
         0    2.0000

[~,err] = cholcov(R)

err =

     0

R = [1 1e-10;0 2]

R =

    1.0000    0.0000
         0    2.0000

[~,err] = cholcov(R)

err =

   NaN

KalmanGain
Index exceeds matrix dimensions.

Error in KalmanGain (line 71)
    optimal_est_y(1,count) = temp(1,1)+temp(1,2)+temp(2,1)+temp(2,2);
 
plot(optimal_est_x(1,count),optimal_est_y(1,count),'ro--')
clf
plot(optimal_est_x(1,count),optimal_est_y(1,count),'ro--')
temp

temp =

  -Inf

multiplier

multiplier =

     0

help mvnpdf
 mvnpdf Multivariate normal probability density function (pdf).
    Y = mvnpdf(X) returns the probability density of the multivariate normal
    distribution with zero mean and identity covariance matrix, evaluated at
    each row of X.  Rows of the N-by-D matrix X correspond to observations or
    points, and columns correspond to variables or coordinates.  Y is an
    N-by-1 vector.
 
    Y = mvnpdf(X,MU) returns the density of the multivariate normal
    distribution with mean MU and identity covariance matrix, evaluated
    at each row of X.  MU is a 1-by-D vector, or an N-by-D matrix, in which
    case the density is evaluated for each row of X with the corresponding
    row of MU.  MU can also be a scalar value, which mvnpdf replicates to
    match the size of X.
 
    Y = mvnpdf(X,MU,SIGMA) returns the density of the multivariate normal
    distribution with mean MU and covariance SIGMA, evaluated at each row
    of X.  SIGMA is a D-by-D matrix, or an D-by-D-by-N array, in which case
    the density is evaluated for each row of X with the corresponding page
    of SIGMA, i.e., mvnpdf computes Y(I) using X(I,:) and SIGMA(:,:,I).
    If the covariance matrix is diagonal, containing variances along the 
    diagonal and zero covariances off the diagonal, SIGMA may also be
    specified as a 1-by-D matrix or a 1-by-D-by-N array, containing 
    just the diagonal. Pass in the empty matrix for MU to use its default 
    value when you want to only specify SIGMA.
 
    If X is a 1-by-D vector, mvnpdf replicates it to match the leading
    dimension of MU or the trailing dimension of SIGMA.
 
    Example:
 
       mu = [1 -1]; Sigma = [.9 .4; .4 .3];
       [X1,X2] = meshgrid(linspace(-1,3,25)', linspace(-3,1,25)');
       X = [X1(:) X2(:)];
       p = mvnpdf(X, mu, Sigma);
       surf(X1,X2,reshape(p,25,25));
 
    See also mvtpdf, mvncdf, mvnrnd, normpdf.

    Reference page for mvnpdf

help lmvnpdf
--- help for mvnpdf ---

 mvnpdf Multivariate normal probability density function (pdf).
    Y = mvnpdf(X) returns the probability density of the multivariate normal
    distribution with zero mean and identity covariance matrix, evaluated at
    each row of X.  Rows of the N-by-D matrix X correspond to observations or
    points, and columns correspond to variables or coordinates.  Y is an
    N-by-1 vector.
 
    Y = mvnpdf(X,MU) returns the density of the multivariate normal
    distribution with mean MU and identity covariance matrix, evaluated
    at each row of X.  MU is a 1-by-D vector, or an N-by-D matrix, in which
    case the density is evaluated for each row of X with the corresponding
    row of MU.  MU can also be a scalar value, which mvnpdf replicates to
    match the size of X.
 
    Y = mvnpdf(X,MU,SIGMA) returns the density of the multivariate normal
    distribution with mean MU and covariance SIGMA, evaluated at each row
    of X.  SIGMA is a D-by-D matrix, or an D-by-D-by-N array, in which case
    the density is evaluated for each row of X with the corresponding page
    of SIGMA, i.e., mvnpdf computes Y(I) using X(I,:) and SIGMA(:,:,I).
    If the covariance matrix is diagonal, containing variances along the 
    diagonal and zero covariances off the diagonal, SIGMA may also be
    specified as a 1-by-D matrix or a 1-by-D-by-N array, containing 
    just the diagonal. Pass in the empty matrix for MU to use its default 
    value when you want to only specify SIGMA.
 
    If X is a 1-by-D vector, mvnpdf replicates it to match the leading
    dimension of MU or the trailing dimension of SIGMA.
 
    Example:
 
       mu = [1 -1]; Sigma = [.9 .4; .4 .3];
       [X1,X2] = meshgrid(linspace(-1,3,25)', linspace(-3,1,25)');
       X = [X1(:) X2(:)];
       p = mvnpdf(X, mu, Sigma);
       surf(X1,X2,reshape(p,25,25));
 
    See also mvtpdf, mvncdf, mvnrnd, normpdf.

    Reference page for mvnpdf


dbquit
normpdf(-5,0,1)

ans =

   1.4867e-06

normpdf(-50,0,1)

ans =

     0

log(2*pi*1)/2-1/2*50^2/1

ans =

  -1.2491e+03

exp(log(2*pi*1)/2-1/2*50^2/1)

ans =

     0

log(exp(log(2*pi*1)/2-1/2*50^2/1))

ans =

  -Inf

KalmanGain
