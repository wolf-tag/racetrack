using System;
using System.Collections;
using System.Collections.Generic;

// solution for vertrac't racetrack 
// https://www.heise.de/ct/artikel/vertrac-t-2-c-t-Racetrack-2844152.html
// accelerate a car so that it reaches the end point in a track with obstacles using a minimum number of steps

// author: Wolfgang Täger
// improved version after deadline using ushort array for storage (used as bit array) and forward/backward search

static class Program
{
    static void Main(string[] args){
		// start and end point
		int xa=120,ya=180,xz=320,yz=220;
		
		// manual limitation of search space
		int x1= 120, x2=359, dx=x2-x1+1;
		int y1= 140, y2=359, dy=y2-y1+1;
		int vx1=-20, vx2=40, dvx=vx2-vx1+1,vx,vy;
		int vy1=-40, vy2=50, dvy=vy2-vy1+1;
		int maxiter=20;
		
		// Bit Array Data Storage (implemented as ushort Array)
		int intbits=16;
		int nint = (int) Math.Ceiling((double)dvy/intbits);	// how many ints needed for all vy 
		int nint2 = nint*dvx;			// how many ints needed for all (vx,vy) pairs

		// main Array comprises all points reachable in iter. 
		// iter, x and y are direct indexes (more precisely we use x-x1 and y-y1), vx and vy are packed into last dimension	
		ushort[,,,] Arr=new ushort[maxiter,dx,dy,nint2];	// var. must have at least intbits

		
		// as vx/vy are packed, we use lookups (vx,vy) => (pos,bitpos) and (pos,bitpos)=>(vx,vy)
		int[,] map_pos=new int[dvx,dvy]; int[,] map_bit=new int[dvx,dvy];
		int[,] map_vx=new int[nint2,intbits]; int[,] map_vy=new int[nint2,intbits];
		int ind=0;
		for (vx=vx1;vx<=vx2;vx++)
			for (int n=0; n<nint; n++){
				for (int b=0; b<intbits;b++){
					//Console.WriteLine("Map (vx={0},vy==>{1}) to (Pos={2},Bitpos={3})",vx,n*intbits+b + vy1,ind,b);

					map_pos[vx-vx1,n*intbits+b]=ind; 
					map_bit[vx-vx1,n*intbits+b]=b;
					
					map_vx[ind,b]=vx;
					map_vy[ind,b]=n*intbits+b + vy1;
					if (n*intbits+b>=dvy-1) break;
					
				}
				ind++;
			}
		
		
		// as acceleration ax/ay is limited (ax^2+ay^2<=100), not all 21*21 possibilities are allowed, so we limit to those
		// prepare accel lookup 
		int accs=0;
		int[] ax=new int[21*21];	// -10..10 => 21 possible values for acceleration x
		int[] ay=new int[21*21];	// idem for y
		for (int a=-10;a<=10;a++)
			for (int b=-10;b<=10;b++)
				if (a*a+b*b<=100) { ax[accs]=a;ay[accs++]=b;}


		// Mark Start and End position in array Arr
		// Start: iter=0, x=xa, y=ya, vx=0, vy=0
		
		// as we do not know how many iterations are needed, we use iter=1, and proceed by a funny path forward 0,2,4,... backward 1,3,5, ...
		// End: iter=1, x=xz, y=yz, vx=0, vy=0			
		
		// Note: we shift all points (x,y) by -(x1,y1) and the speeds (vx,vy) by -(vx1,vy1) to get index values starting with zero
		int pos0 = map_pos[0-vx1,0-vy1];                	// map vx=0, vy=0 to a value set in last dimension of Arr
		int bitpos0 = map_bit[0-vx1,0-vy1];                	
		Arr[0,xa-x1,ya-y1,pos0]=(ushort) (1<<bitpos0);       // mark Start
		Arr[1,xz-x1,yz-y1,pos0]=(ushort) (1<<bitpos0);       // mark End
		

		
		/// FORWARD SECTION

		bool found=false;
		int diter=2;
		int turniter=0;
		
		for (int iter=2; (!found || iter>0) && iter<maxiter;iter+=diter){					// forward and backward in one loop, therefore increase +2
			ushort[,,] TmpArray=new ushort[dx,dy,nint2];
			if (diter>0)
				Console.WriteLine("Forward Iter {0}=>{1}",iter-diter,iter);
			else
				Console.WriteLine("Forward after Rendezvous Iter {0}=>Tmp",iter-diter);
			// go through all x,y, check if vx/vy-pairs exist for pair x/y by looking into packed array
			for (int x = x1; x<=x2;x++){
				for (int y = y1; y<=y2; y++){
					vx=vx1;
					ind=0;
					for (int i= 1; i<=nint2/nint;i++){		//50
						vy=vy1;
						for (int j=1; j<=nint;j++){
							ushort s=Arr[iter-diter,x-x1,y-y1,ind++];
							
							if (s==0) {vy=vy+intbits; continue;}			// if no bit is set, there is no vx/vy combination for x/y, so we continue
							for (int shi=1; shi <= intbits; shi++){			// otherwise we go through the bits by taking LSB and right shift 
								bool bit=( (s%2)>0);
								s>>=1;
								if (bit){  	/* pos x,y,vx,vy exists */
									int vx_end,vy_end,x_end,y_end;
									for (int aind=0; aind<accs;aind++){ 	// go through all allowable accel vectors
										vx_end=vx + ax[aind];vy_end=vy + ay[aind];
										// check manual limits for vx/vy to filter out obviously wrong cases
										if (vx_end<vx1 || vx_end >vx2 || vy_end<vy1 || vy_end >vy2 ) continue; 
										
										x_end=x + vx_end; y_end=y + vy_end;
										// check manual limits for x/y to filter out obviously wrong cases
										if (x_end<x1 || x_end >x2 || y_end<y1 || y_end >y2 ) continue;
										
										if (coll.collision (x,y,x_end,y_end)) continue;	// Kollision mit Wänden Prüfen

										// as all tests were positive, we keep the result as a candidate of the track
										int pos = map_pos[vx_end-vx1,vy_end-vy1]; int bitpos = map_bit[vx_end-vx1,vy_end-vy1];
										if (diter>0)	// during search, diter>0, so we keep result
											Arr[iter, x_end-x1,y_end-y1,pos] |= (ushort)(1<<bitpos);
										else			// once solution is found, diter<0 to collect all path in a backward manner
											TmpArray[x_end-x1,y_end-y1,pos] |= (ushort)(1<<bitpos);
										
									}
								}
								vy++;
							}
						}
						vx++;
					}
				}
			}
			if(diter<0){
				Console.WriteLine("{0} AND Tmp to  {1}",iter, iter);		// print the elements of the shortest pathes (by going backwards)
				for (int x = x1; x<=x2;x++)
					for (int y = y1; y<=y2; y++)
						for (int n=0;n<nint2;n++)
							Arr[iter, x-x1,y-y1,n] &= TmpArray[x-x1,y-y1,n];	// remove all candidates that are not on the shortest path
				
			}
			// check if we reached the goal by one of the candidates
			// as we search from start and end, this means, we found one or several solutions if forward and backward search resulted in same x,y,vx,vy
			for (int x = x1; x<=x2&& !found;x++)
				for (int y = y1; y<=y2 && !found; y++)
					for (int n=0;n<nint2 && !found;n++)
						if ( (Arr[iter, x-x1,y-y1,n] & Arr[iter-1, x-x1,y-y1,n])>0){
							found=true;	
							turniter=iter-1;
							Console.WriteLine("solution found");
							diter=-2;			// switch to backwards mode for finding all paths
						}
			if (found && iter-1==turniter){
				Console.WriteLine("Iter {0} AND {1} => {2},{3}",iter-1,iter, iter-1, iter);
				for (int x = x1; x<=x2;x++)
					for (int y = y1; y<=y2; y++){
						for (int n=0;n<nint2;n++){
							Arr[iter-1, x-x1,y-y1,n] &= Arr[iter, x-x1,y-y1,n];
							Arr[iter, x-x1,y-y1,n] = Arr[iter-1, x-x1,y-y1,n];
						}
						
					}
				//continue;			// here we skip backward, as we have to do forward again
				iter--;				// backward after RDV shall start with the same iter as we currently are
			}


			/// BACKWARD SECTION - this is mainly a copy of the FORWARD SECTION
			Array.Clear(TmpArray,0,TmpArray.Length);		
			

			if (diter>0)
				Console.WriteLine("Backward Iter {0}=>{1}",iter-diter/2,iter+diter/2);
			else
				Console.WriteLine("Backward after Rendezvous Iter {0}=>Tmp",iter-diter/2);
			
			for (int x = x1; x<=x2;x++){
				for (int y = y1; y<=y2; y++){
					vx=vx1;
					ind=0;
					for (int i= 1; i<=nint2/nint;i++){		//50
						vy=vy1;
						for (int j=1; j<=nint;j++){
							ushort s=Arr[iter-diter/2,x-x1,y-y1,ind++];		// the iteration number decreases for the backward path during search
							if (s==0) {vy=vy+intbits; continue;}
							for (int shi=1; shi <= intbits; shi++){
								bool bit=( (s%2)>0);
								s>>=1;
								if (bit){  	/* pos x,y,vx,vy exists */
									int vx_end,vy_end,x_end,y_end;
									for (int aind=0; aind<accs;aind++){ // all allowable accel vectors
										x_end=x - vx; y_end=y - vy;
										if (x_end<x1 || x_end >x2 || y_end<y1 || y_end >y2 ) continue;

										vx_end=vx - ax[aind];vy_end=vy - ay[aind];
										if (vx_end<vx1 || vx_end >vx2 || vy_end<vy1 || vy_end >vy2 ) continue;
										
										if (coll.collision (x,y,x_end,y_end)) continue;	// Kollision mit Wänden Prüfen
										
										int pos = map_pos[vx_end-vx1,vy_end-vy1]; int bitpos = map_bit[vx_end-vx1,vy_end-vy1];

										if (diter>0)
											Arr[iter+diter/2, x_end-x1,y_end-y1,pos] |= (ushort)(1<<bitpos);
										else
											TmpArray[x_end-x1,y_end-y1,pos] |= (ushort)(1<<bitpos);										
									}
								}
								vy++;
							}
						}
						vx++;
					}
				}
			}
			if(diter<0){
				Console.WriteLine("{0} AND Tmp to  {1}",iter+diter/2,iter+diter/2);
				for (int x = x1; x<=x2;x++)
					for (int y = y1; y<=y2; y++)
						for (int n=0;n<nint2;n++)
							Arr[iter+diter/2, x-x1,y-y1,n] &= TmpArray[x-x1,y-y1,n];
				
			}
			for (int x = x1; x<=x2&& !found;x++)
				for (int y = y1; y<=y2 && !found; y++)
					for (int n=0;n<nint2 && !found;n++)
						if ( (Arr[iter+1, x-x1,y-y1,n] & Arr[iter, x-x1,y-y1,n])>0){
							found=true;
							turniter=iter;
							Console.WriteLine("solution found");
							diter=-2;
						}

			if (found && iter==turniter){
				Console.WriteLine("Iter {0} AND {1} => {2},{3}",iter,iter+1, iter, iter+1);

				for (int x = x1; x<=x2;x++)
					for (int y = y1; y<=y2; y++)
						for (int n=0;n<nint2;n++){
							Arr[iter, x-x1,y-y1,n] &= Arr[iter+1, x-x1,y-y1,n];
							Arr[iter+1, x-x1,y-y1,n] = Arr[iter, x-x1,y-y1,n];
						}
			}
			
		}

		// In principle the problem is solved, we have retained only points iter,x,y,vx,vy which are parts of the shortest solutions
		// now we want to show and count all solutions
		// this can be done by following the retained points on the pathes (we stored vx,vy for them to get to the next points)
		// this is done in a factorised approach, i.e. if several paths join in a point iter,x,y,vx,vy, the number of solutions to this points add up
		
		Dictionary<string,UInt64> count = new Dictionary<string,UInt64 >();
		
		String point=hashme.gethash(0,xa,ya,0,0);	// Start point as hash (here hash is just a unique identifier for the combination, not a cryptographic hash)
		count.Add(point,1);							// Put in dictionary for start point one solution as start value
		diter=2;
		int ii=0;
		int line=0;
		Console.WriteLine("turniter={0}",turniter);
		Console.WriteLine();
		Console.WriteLine("id,paths to here,iter,x,y,vx,xy");
		if (found){
			// just follow the path iter,x,y and check which vx/vy we kept			
			for (int iter=0;iter>=0;iter+=diter){
				int nextiter;
				nextiter=iter+diter;
				Console.WriteLine("iter={0}",iter);
				Console.WriteLine("nextiter={0}",nextiter);
				for (int x = x1; x<=x2;x++){
					for (int y = y1; y<=y2; y++){
						vx=vx1;
						ind=0;
						for (int i= 1; i<=nint2/nint;i++){		
							vy=vy1;
							for (int j=1; j<=nint;j++){
								ushort s=Arr[iter,x-x1,y-y1,ind++];
								if (s==0) {vy=vy+intbits; continue;}
								for (int shi=1; shi <= intbits; shi++){
									bool bit=( (s%2)>0);
									s>>=1;
									if (bit){  	/* pos x,y,vx,vy exists */							
										// !!
										string point_from=hashme.gethash(ii,x,y,vx,vy);
										if (count.ContainsKey(point_from))
											Console.Write("{0},{1},{2},{3},{4},{5},{6}",line++, count[point_from],ii,x,y,vx,vy);
										else
											Console.Write("{0},{1},{2},{3},{4},{5},{6}",line++, "error" ,ii,x,y,vx,vy); // should not happen, just a check
										// there is actually no need to try all accel vectors, so this code should be simplified
										for (int aind=0; aind<accs;aind++){ // all allowable accel vectors
											int vx_end=vx + ax[aind];int vy_end=vy + ay[aind];
											if (vx_end<vx1 || vx_end >vx2 || vy_end<vy1 || vy_end >vy2 ) continue;
											
											int x_end=x + vx_end; int y_end=y + vy_end;
											if (x_end<x1 || x_end >x2 || y_end<y1 || y_end >y2 ) continue;
											
											if (coll.collision (x,y,x_end,y_end)) continue;	// Kollision mit Wänden Prüfen
											int pos = map_pos[vx_end-vx1,vy_end-vy1]; int bitpos = map_bit[vx_end-vx1,vy_end-vy1];
											if (nextiter>0)
												if ((Arr[nextiter, x_end-x1,y_end-y1,pos] & (ushort)(1<<bitpos))>0){
													Console.Write(",,{0},{1}",ax[aind],ay[aind]);
													//string point_from=hashme.gethash(ii,x,y,vx,vy);
													string point_to=hashme.gethash(ii+1,x_end,y_end,vx_end,vy_end);
													if (count.ContainsKey(point_to)) 
														count[point_to]=count[point_to]+count[point_from];	// add number of solutions for different paths
													else 
														count.Add(point_to, count[point_from]);		//
													
												}
										}
										Console.WriteLine("");
									}
									vy++;
								}
							}
							vx++;
						}
					}
				}
				ii++;
					
				if (iter+diter>turniter){
					diter=-2;
					iter+=3;
				}
			}
		}
	}

}
public static class coll{
	public static bool collision (int x,int y,int x_end, int y_end){
		
		// wall y=200, x=100..200
		if ((y-200)*(y_end-200)<=0){		// crosses or touches y=200
			if (y_end-y==0){				// y constant 200
				if (!((x_end<100 && x<100) || (x_end>200 && x>200))) return true;
			}
			else{
				double xc=((double)(200-y))/(y_end-y)*(x_end-x)+x;
				if (xc>=100 && xc<=200) return true;
			}
		}
		
		// wall x=250, y=200..300
		if ((x-250)*(x_end-250)<=0){		// crosses or touches x=250
			if (x_end-x==0){				// x constant 250
				if (!((y_end<200 && y<200) || (y_end>300 && y>300))) return true;
			}
			else{
				double yc=((double)(250-x))/(x_end-x)*(y_end-y)+y;
				if (yc>=200 && yc<=300) return true;
			}
		}
		
		// wall x=300, y=100..300
		if ((x-300)*(x_end-300)<=0){		// crosses or touches x=300
			if (x_end-x==0){				// x constant 300
				if (!((y_end<100 && y<100) || (y_end>300 && y>300))) return true;
			}
			else {
				double yc=((double)(300-x))/(x_end-x)*(y_end-y)+y;
				if (yc>=100 && yc<=300) return true;
			}
		}
		return false;
	}
}

public static class hashme{
	public static string gethash (int iter, int x,int y,int vx, int vy){
		return Convert.ToString(iter) + "#" + Convert.ToString(x) + "#" + Convert.ToString(y) + "#" + Convert.ToString(vx) + "#" + Convert.ToString(vy); 
	}
}

	
		
