% gets only the Basisfunctions for multiple joints and timesteps
function PSI_X = getBasisFunctionsMultipleJointsAndTimesteps(n,T,D,nder)
%n=number of basisfunctions
%T=number of Timesteps
%D=number of joints
%nder= number of derivatives
    psi = getBasisFunctions(n,T,nder);
    PSI_X = zeros(nder*D*T,n*D);
    for t=1:T
       actual = squeeze(psi(:,:,t));
       for j=1:D  
           PSI_X(nder*(j-1)+1+nder*D*(t-1):nder*j+nder*D*(t-1),(j-1)*n+1:n*j) = actual;
       end
    end
end