package com.mantono;

import java.awt.Point;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.DeathEvent;
import robocode.HitRobotEvent;
import robocode.MessageEvent;
import robocode.RobotDeathEvent;
import robocode.RoundEndedEvent;
import robocode.ScannedRobotEvent;
import robocode.TeamRobot;

public abstract class AbstractRobot extends TeamRobot
{
	/**
	 * @author Anton &Ouml;sterberg (anton.osterberg@gmail.com)
	 */
	private final Map<String,List<Double>> velocityRecord = new HashMap<String,List<Double>>();
	public static final int UP = 0;
	public static final int RIGHT = 1;
	public static final int DOWN = 2;
	public static final int LEFT = 3;
	private double a, b, c;
	private double distanceRight, distanceDown, distanceLeft, distanceUp;
	private int bulletHits = 0;
	private int bulletMisses = 0;

	/**
	 * Method for avoiding walls.
	 * @param perimiter decides at which distance the robot should start turning for walls it is approaching.
	 * @return true if the robot had to make a turn to avoid a wall, else false.
	 */
	public boolean turnForWall(double perimiter)
	{
		double distance = distanceToWall();
		if(distance < perimiter)
		{
			if(distanceLeftBeam() < distanceRightBeam())
				setTurnRight(800/distance);
			else
				setTurnLeft(800/distance);
			execute();
			return true;
		}
		return false;
	}

	/**
	 * Will steer the robot towards the coordinates given as parameters.
	 * @param x coordinate for the destination.
	 * @param y coordinate for the destination.
	 * @return true when position is reached.
	 */
	public boolean goTo(double x, double y)
	{
		while(distanceTo(x, y) > (getHeight()/2 + 5))
		{
			setHeading(bearingTo(x, y));
			setAhead(10);
			execute();
		}
		return true;
	}

	/**
	 * Sets the absolute heading. Heading will be +/- 1 from the given heading.
	 * @param heading the new heading for the robot.
	 * @return true when the heading is reached.
	 */
	public boolean setHeading(double heading)
	{
		while(getHeading() - heading > 1.0 || getHeading() - heading < -1.0)
			turnLeft(relativeAngle(getHeading() - heading));
		return true;
	}

	/**
	 * Sets the absolute heading.
	 * @param heading is the new heading for the robot.
	 * @param precision sets the threshold for how much the heading is allowed to differ.
	 * @return true when the heading is reached.
	 */
	public boolean setHeading(double heading, double precision)
	{
		while(getHeading() - heading > precision || getHeading() - heading < precision)
			turnLeft(precision);
		return true;
	}

	/**
	 * Calculates the distance from the robot to the coordinate.
	 * @param x coordinate for the position that distance should be measured to.
	 * @param y coordinate for the position that distance should be measured to.
	 * @return the distance.
	 */
	public double distanceTo(double x, double y)
	{
		final double deltaX = getX() - x;
		final double deltaY = getY() - y;
		final double pwX = Math.pow(deltaX, 2);
		final double pwY = Math.pow(deltaY, 2);
		return Math.sqrt(pwX + pwY);
	}

	public double relativeBearingTo(double x, double y)
	{
		final double deltaX = x - getX();
		final double deltaY = y - getY();
		final double hypotenuse = distanceTo(x, y);
		final double arcSin = Math.toDegrees(Math.asin(deltaX / hypotenuse));
		double bearing = 0;

		if(deltaX > 0 && deltaY > 0)
			bearing = arcSin;
		else if(deltaX < 0 && deltaY > 0)
			bearing = 360 + arcSin;
		else if(deltaX < 0 && deltaY < 0)
			bearing = 180 - arcSin;

		if(bearing > 180)
			bearing -= 180;
		if(bearing < -180)
			bearing += 180;
		return bearing;
	}

	public double bearingTo(double x, double y)
	{
		final double deltaX = x - getX();
		final double deltaY = y - getY();
		final double hypotenuse = distanceTo(x, y);
		final double arcSin = Math.toDegrees(Math.asin(deltaX / hypotenuse));
		double bearing = 0;

		if(deltaX > 0 && deltaY > 0)
			bearing = arcSin;
		else if(deltaX < 0 && deltaY > 0)
			bearing = 360 + arcSin;
		else if(deltaX < 0 && deltaY < 0)
			bearing = 180 - arcSin;

		return bearing;
	}

	public double distanceFromOrigo(double x, double y)
	{
		final double deltaX = 0 - x;
		final double deltaY = 0 - y;
		final double pwX = Math.pow(deltaX, 2);
		final double pwY = Math.pow(deltaY, 2);
		return Math.sqrt(pwX + pwY);
	}

	public boolean closestWallsIsOnLeftSide()
	{
		final double heading = getHeading();
		if(heading >= 0 && heading < 110)
			return getX() > getBattleFieldWidth() - getX();
		else if(heading >= 70 && heading < 180)
			return getY() > getBattleFieldHeight() - getY();
		else if(heading >= 180 && heading < 290)
			return getX() < getBattleFieldWidth() - getX();
		else if(heading >= 250 && heading < 360)
			return getY() < getBattleFieldHeight() - getY();
		return false;
	}

	private void calculateDistances()
	{
		distanceRight = getBattleFieldWidth() - getX();
		distanceDown = getY();
		distanceLeft = getX();
		distanceUp = getBattleFieldHeight() - getY();
	}

	/**
	 * Calculates the distance to the robot and the wall the robot is currently heading towards.
	 * @return the distance between the robot and the wall.
	 */
	public double distanceToWall()
	{
		calculateDistances();
		final double heading = getHeading();

		if(heading > 0 && heading <= 90)
			a = (distanceRight < distanceUp) ? distanceRight : distanceUp;
		else if(heading > 90 && heading <= 180)
			a = (distanceRight < distanceDown) ? distanceRight : distanceDown;
		else if(heading > 180 && heading <= 270)
			a = (distanceDown < distanceLeft) ? distanceDown : distanceLeft;
		else
			a = (distanceLeft < distanceUp) ? distanceLeft : distanceUp;

		double theta = heading;
		while(theta > 45)
			theta -= 90;
		if(theta < 0)
			theta *= -1;
		theta = Math.toRadians(theta);
		b = Math.tan(theta)*a;
		c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));

		return c;
	}

	public double distanceLeftBeam()
	{
		calculateDistances();

		double leftBeam = getHeading() - 90;
		if(leftBeam < 0)
			leftBeam += 360;

		if(leftBeam > 0 && leftBeam <= 90)
			a = (distanceRight < distanceUp) ? distanceRight : distanceUp;
		else if(leftBeam > 90 && leftBeam <= 180)
			a = (distanceRight < distanceDown) ? distanceRight : distanceDown;
		else if(leftBeam > 180 && leftBeam <= 270)
			a = (distanceDown < distanceLeft) ? distanceDown : distanceLeft;
		else
			a = (distanceLeft < distanceUp) ? distanceLeft : distanceUp;

		double theta = leftBeam;
		while(theta > 45)
			theta -= 90;
		if(theta < 0)
			theta *= -1;
		theta = Math.toRadians(theta);
		b = Math.tan(theta)*a;
		c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));
		return c;
	}

	public double distanceRightBeam()
	{
		calculateDistances();

		double rightBeam = getHeading() + 90;
		if(rightBeam > 360)
			rightBeam -= 360;

		if(rightBeam > 0 && rightBeam <= 90)
			a = (distanceRight < distanceUp) ? distanceRight : distanceUp;
		else if(rightBeam > 90 && rightBeam <= 180)
			a = (distanceRight < distanceDown) ? distanceRight : distanceDown;
		else if(rightBeam > 180 && rightBeam <= 270)
			a = (distanceDown < distanceLeft) ? distanceDown : distanceLeft;
		else
			a = (distanceLeft < distanceUp) ? distanceLeft : distanceUp;

		double theta = rightBeam;
		while(theta > 45)
			theta -= 90;
		if(theta < 0)
			theta *= -1;
		theta = Math.toRadians(theta);
		b = Math.tan(theta)*a;
		c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));
		return c;
	}



	public double getAverageVelocity(String targetName)
	{
		if(!velocityRecord.containsKey(targetName))
			return 20;
		List<Double> robotVelocityRecord = velocityRecord.get(targetName);
		int amountOfRecords = robotVelocityRecord.size();
		double averageVelocity = 0;
		int i;
		for(i = 0; i < 20 && i < amountOfRecords; i++)
			averageVelocity += robotVelocityRecord.get(amountOfRecords-1-i);
		return averageVelocity/i;
	}

	public void updateVelocityRecord(ScannedRobotEvent robot)
	{
		if(!velocityRecord.containsKey(robot.getName()))
			velocityRecord.put(robot.getName(), new ArrayList<Double>());
		List<Double> robotVelocityRecord = velocityRecord.get(robot.getName());
		robotVelocityRecord.add(Math.abs(robot.getVelocity()));
	}
	
	@Override
	public void onMessageReceived(MessageEvent event)
	{
		Serializable message = event.getMessage();
		if(message instanceof ScannedRobotEvent)
			updateVelocityRecord((ScannedRobotEvent) message);
	}
	
	@Override
	public void onScannedRobot(ScannedRobotEvent target)
	{
		if(!isTeammate(target.getName()))
		{
			updateVelocityRecord(target);
			try
			{
				broadcastMessage(target);
			}
			catch(IOException exception)
			{
				System.err.println("Could not broadcast message");
				exception.printStackTrace();
			}
		}
	}

	@Override
	public void onRobotDeath(RobotDeathEvent robot)
	{
		velocityRecord.remove(robot.getName());		
	}

	public String getSlowestRobot()
	{
		String slowest = null;
		for(String robot : velocityRecord.keySet())
		{
			if(slowest == null || getAverageVelocity(robot) < getAverageVelocity(slowest))
				slowest = robot;
		}
		return slowest;
	}

	public double chanceOfHit(ScannedRobotEvent target)
	{
		final double distance = target.getDistance();
		final double relativeHeading = Math.abs(target.getBearingRadians() - getGunHeadingRadians());
		final double relativeDirection = Math.abs(target.getHeadingRadians() - getGunHeadingRadians());
		final double speed = Math.abs(target.getVelocity());
		
		//TODO this can certainly be improved...
		
		return 10000/((distance/2 + relativeHeading*2 + relativeDirection)*speed+1);
	}

	@Override
	public void onHitRobot(HitRobotEvent event)
	{
		if(isTeammate(event.getName()))
			turnLeft(90);
		else
			{
				turnRight(relativeAngle(event.getBearing()));
				fire(3);
			}
	}

	public void alignGunToCenter()
	{
		turnGunRight(getHeading() - getGunHeading());
	}

	public double getGunCenter()
	{
		return (getHeading() - getGunHeading());
	}

	public double relativeAngle(double angle)
	{
		while(angle >  180)
			angle -= 360;
		while(angle < -180)
			angle += 360;
		return angle;
	}

	/**
	 * Gives the exact center of the battlefield.
	 * @return a <tt>Point</tt> with the coordinates corresponding to the center of the battlefield.
	 */
	public Point getCenterOfBattleField()
	{
		return new Point((int) Math.round(getBattleFieldWidth()/2), (int) Math.round(getBattleFieldHeight()/2.0));
	}


	/**
	 * Finds the wall closest to the robot.
	 * @return an int representing the wall which is closest to the robot.
	 */
	public int closestWall()
	{
		calculateDistances();
		double[] distances = new double[] {distanceUp, distanceDown, distanceLeft, distanceRight};
		Arrays.sort(distances);
		double min = distances[0];

		if(min == distanceLeft)
			return LEFT;
		if(min == distanceRight)
			return RIGHT;
		if(min == distanceUp)
			return UP;
		if(min == distanceDown)
			return DOWN;
		return -1;
	}

	public double setBulletVelocity(double velocity)
	{
		return (velocity - 20) / -3;
	}

	public double getGunHeadingRelativeTo(ScannedRobotEvent target)
	{
		return getHeading() + target.getBearing() - getGunHeading();
	}

	public double getMovementCompensation(ScannedRobotEvent target)
	{
		double relativeHeading = target.getBearing() + (target.getHeading()/getHeading());
		double movementCompensation = relativeHeading/8*(target.getVelocity()/4);
		movementCompensation *= target.getDistance()/400;
		return movementCompensation;
	}

	@Override
	public void onBulletMissed(BulletMissedEvent event)
	{
		bulletMisses++;

	}

	@Override
	public void onBulletHit(BulletHitEvent event)
	{
		bulletHits++;
	}

	@Override
	public void onRoundEnded(RoundEndedEvent event)
	{
		showStats();
	}

	@Override
	public void onDeath(DeathEvent event)
	{
		showStats();
	}

	public void showStats()
	{
		int bulletsFired = bulletHits + bulletMisses;
		double accuracy = (double) bulletHits/bulletsFired;
		accuracy *= 100;
		final String row = "-----------------------------------------------------------";
		System.out.println(row);
		System.out.println("Fire accuracy: " + accuracy + "%");
		System.out.println(row);
	}

}

