function z = sphere2D(x, y, goal)
    z = 0.8*(x - goal(1)).^2 + 0.8*(y - goal(2)).^2;
end