function frame = disframe(T,L,m)
    %disegna una terna rispetto a world
%calcoliamo i punti in coordinate world e poi disegnamo gli assi

P = T*[[0 0 0 1]' [L 0 0 1]' [0 L 0 1]' [0 0 L 1]'];


x =line( P(1,[1 2]) , P(2,[1 2]) , P(3,[1 2]) , 'color','r','linewidth',2,'marker',m);    % -----> asse X rosso
y =line( P(1,[1 3]) , P(2,[1 3]) , P(3,[1 3]) , 'color','g','linewidth',2,'marker',m);    % -----> asse Y verde
z =line( P(1,[1 4]) , P(2,[1 4]) , P(3,[1 4]) , 'color','b','linewidth',2,'marker',m);    % -----> asse Z blu

frame = [x,y,z];
end
