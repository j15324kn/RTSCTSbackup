//#include <ip.h>

#include <aodv/aodv.h>
#include <aodv/aodv_packet.h>
#include <random.h>
#include <cmu-trace.h>
//#include <energy-model.h>

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#define max(a,b)        ( (a) > (b) ? (a) : (b) )
#define CURRENT_TIME    Scheduler::instance().clock()

//#define DEBUG
//#define ERROR

//tuika------------------------------//
//#define KIZON 
#define PROP
#define DST
//#define MESS1 
//#define MESS2
//#define MISU
//#define REPLY //中間REPLYを行う場合は外す
//------------------------------//

#ifdef DEBUG
static int route_request = 0;
static int limit_route_request = 0;
static int route_request = 0;
#endif

/*
  TCL Hooks
*/

int hdr_aodv::offset_;
static class AODVHeaderClass : public PacketHeaderClass {
public:
        AODVHeaderClass() : PacketHeaderClass("PacketHeader/AODV",
                                              sizeof(hdr_all_aodv)) {
	  bind_offset(&hdr_aodv::offset_);
	} 
} class_rtProtoAODV_hdr;

static class AODVclass : public TclClass {
public:
        AODVclass() : TclClass("Agent/AODV") {}
        TclObject* create(int argc, const char*const* argv) {
          assert(argc == 5);
          //return (new AODV((nsaddr_t) atoi(argv[4])));
	  return (new AODV((nsaddr_t) Address::instance().str2addr(argv[4])));
        }
} class_rtProtoAODV;

int
AODV::command(int argc, const char*const* argv) {
  if(argc == 2) {
  Tcl& tcl = Tcl::instance();
    
    if(strncasecmp(argv[1], "id", 2) == 0) {
      tcl.resultf("%d", index);
      return TCL_OK;
    }
    
    if(strncasecmp(argv[1], "start", 2) == 0) {
      btimer.handle((Event*) 0);

#ifndef AODV_LINK_LAYER_DETECTION
      htimer.handle((Event*) 0);
      ntimer.handle((Event*) 0);
#endif // LINK LAYER DETECTION

      rtimer.handle((Event*) 0);
      return TCL_OK;
     }               

//tuika-misu-----------------------------//
#ifdef MISU
   if(strcmp(argv[1], "show-node-energy") == 0) {
    double my_energy;
    my_energy = thisnode->energy_model()->energy();
//  printf("\n%4.5f(sec)_____ノード[%d]情報:バッテリー残量(my_energy=%4.3f)\n",CURRENT_TIME, index, my_energy);  
    return TCL_OK;
    }

   if(strcmp(argv[1], "show-node-info") == 0) {

    double x_position, y_position, z_position;
    double x_speed, y_speed, z_speed;
    double my_energy;

    thisnode->getLoc(&x_position, &y_position, &z_position);
    thisnode->getVelo(&x_speed, &y_speed, &z_speed);
    my_energy = thisnode->energy_model()->energy();
//  printf("\n%4.5f(sec)_____ノード[%d]情報。位置(x_position=%4.3f, y_position=%4.3f, z_position=%4.3f) スピード(x_speed=%4.3f, y_speed=%4.3f, z_speed=%4.3f) バッテリー残量(my_energy=%4.3f)\n",CURRENT_TIME, index, x_position, y_position, z_position, x_speed, y_speed, z_speed, my_energy);  
    return TCL_OK;
    }
#endif //MISU
//------------------------------//


  }
  else if(argc == 3) {
    if(strcmp(argv[1], "index") == 0) {
      index = atoi(argv[2]);
      return TCL_OK;
    }

    else if(strcmp(argv[1], "log-target") == 0 || strcmp(argv[1], "tracetarget") == 0) {
      logtarget = (Trace*) TclObject::lookup(argv[2]);
      if(logtarget == 0)
	return TCL_ERROR;
      return TCL_OK;
    }
    else if(strcmp(argv[1], "drop-target") == 0) {
    int stat = rqueue.command(argc,argv);
      if (stat != TCL_OK) return stat;
      return Agent::command(argc, argv);
    }
    else if(strcmp(argv[1], "if-queue") == 0) {
    ifqueue = (PriQueue*) TclObject::lookup(argv[2]);
      
      if(ifqueue == 0)
	return TCL_ERROR;
      return TCL_OK;
    }
    else if (strcmp(argv[1], "port-dmux") == 0) {
    	dmux_ = (PortClassifier *)TclObject::lookup(argv[2]);
	if (dmux_ == 0) {
		fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__,
		argv[1], argv[2]);
		return TCL_ERROR;
	}
	return TCL_OK;
    }
  }
  return Agent::command(argc, argv);
}

/* 
   Constructor
*/

AODV::AODV(nsaddr_t id) : Agent(PT_AODV),
			  btimer(this), htimer(this), ntimer(this), 
			  rtimer(this), lrtimer(this), rqueue() {
 
                
  index = id;
  seqno = 2;
  bid = 1;

//tuika------------------------------//
//追加変数aodv.hに記載
thisnode = (MobileNode *)(Node::get_node_by_address(id));
N = 0;
Dis = 1;
tempmin = 9999;

//denpaarea = 100; //denpa100m
denpaarea = 150; //denpa150m
//denpaarea = 175; //denpa175m
denpakeisan;
app_distance[100];

/*
  for(int a=0;a<100;a++){
  app_distance[a]=9999;
  } 
  app_time[100];
  for(int b=0;b<100;b++){
      app_time[b]=9999;
  } 
*/
//------------------------------//


  LIST_INIT(&nbhead);
  LIST_INIT(&bihead);

  logtarget = 0;
  ifqueue = 0;
}

/*
  Timers
*/

void
BroadcastTimer::handle(Event*) {
  agent->id_purge();
  Scheduler::instance().schedule(this, &intr, BCAST_ID_SAVE);
}

void
HelloTimer::handle(Event*) {
   agent->sendHello();
   double interval = MinHelloInterval + 
                 ((MaxHelloInterval - MinHelloInterval) * Random::uniform());
   assert(interval >= 0);
   Scheduler::instance().schedule(this, &intr, interval);
}

void
NeighborTimer::handle(Event*) {
  agent->nb_purge();
  Scheduler::instance().schedule(this, &intr, HELLO_INTERVAL);
}

void
RouteCacheTimer::handle(Event*) {
  agent->rt_purge();
#define FREQUENCY 0.5 // sec
  Scheduler::instance().schedule(this, &intr, FREQUENCY);
}

void
LocalRepairTimer::handle(Event* p)  {  // SRD: 5/4/99
aodv_rt_entry *rt;
struct hdr_ip *ih = HDR_IP( (Packet *)p);

   /* you get here after the timeout in a local repair attempt */
   /*	fprintf(stderr, "%s\n", __FUNCTION__); */


    rt = agent->rtable.rt_lookup(ih->daddr());
	
    if (rt && rt->rt_flags != RTF_UP) {
    // route is yet to be repaired
    // I will be conservative and bring down the route
    // and send route errors upstream.
    /* The following assert fails, not sure why */
    /* assert (rt->rt_flags == RTF_IN_REPAIR); */
		
      //rt->rt_seqno++;
      agent->rt_down(rt);
      // send RERR
#ifdef DEBUG
      fprintf(stderr,"Dst - %d, failed local repair\n", rt->rt_dst);
#endif      
    }
    Packet::free((Packet *)p);
}


/*
   Broadcast ID Management  Functions
*/


void
AODV::id_insert(nsaddr_t id, u_int32_t bid) {
BroadcastID *b = new BroadcastID(id, bid);

 assert(b);
 b->expire = CURRENT_TIME + BCAST_ID_SAVE;
 LIST_INSERT_HEAD(&bihead, b, link);
}

/* SRD */
bool
AODV::id_lookup(nsaddr_t id, u_int32_t bid) {
BroadcastID *b = bihead.lh_first;
 
 // Search the list for a match of source and bid
 for( ; b; b = b->link.le_next) {
   if ((b->src == id) && (b->id == bid))
     return true;     
 }
 return false;
}

void
AODV::id_purge() {
BroadcastID *b = bihead.lh_first;
BroadcastID *bn;
double now = CURRENT_TIME;

 for(; b; b = bn) {
   bn = b->link.le_next;
   if(b->expire <= now) {
     LIST_REMOVE(b,link);
     delete b;
   }
 }
}

/*
  Helper Functions
*/

double
AODV::PerHopTime(aodv_rt_entry *rt) {
int num_non_zero = 0, i;
double total_latency = 0.0;

 if (!rt)
   return ((double) NODE_TRAVERSAL_TIME );
	
 for (i=0; i < MAX_HISTORY; i++) {
   if (rt->rt_disc_latency[i] > 0.0) {
      num_non_zero++;
      total_latency += rt->rt_disc_latency[i];
   }
 }
 if (num_non_zero > 0)
   return(total_latency / (double) num_non_zero);
 else
   return((double) NODE_TRAVERSAL_TIME);

}

/*
  Link Failure Management Functions
*/

static void
aodv_rt_failed_callback(Packet *p, void *arg) {
  ((AODV*) arg)->rt_ll_failed(p);
}

/*
 * This routine is invoked when the link-layer reports a route failed.
 */
void
AODV::rt_ll_failed(Packet *p) {
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
aodv_rt_entry *rt;
nsaddr_t broken_nbr = ch->next_hop_;

#ifndef AODV_LINK_LAYER_DETECTION
 drop(p, DROP_RTR_MAC_CALLBACK);
#else 

 /*
  * Non-data packets and Broadcast Packets can be dropped.
  */
  if(! DATA_PACKET(ch->ptype()) ||
     (u_int32_t) ih->daddr() == IP_BROADCAST) {
    drop(p, DROP_RTR_MAC_CALLBACK);
    return;
  }
  log_link_broke(p);
	if((rt = rtable.rt_lookup(ih->daddr())) == 0) {
    drop(p, DROP_RTR_MAC_CALLBACK);
    return;
  }
  log_link_del(ch->next_hop_);

#ifdef AODV_LOCAL_REPAIR
  /* if the broken link is closer to the dest than source, 
     attempt a local repair. Otherwise, bring down the route. */


  if (ch->num_forwards() > rt->rt_hops) {
    local_rt_repair(rt, p); // local repair
    // retrieve all the packets in the ifq using this link,
    // queue the packets for which local repair is done, 
    return;
  }
  else	
#endif // LOCAL REPAIR	

  {
    drop(p, DROP_RTR_MAC_CALLBACK);
    // Do the same thing for other packets in the interface queue using the
    // broken link -Mahesh
while((p = ifqueue->filter(broken_nbr))) {
     drop(p, DROP_RTR_MAC_CALLBACK);
    }	
    nb_delete(broken_nbr);
  }

#endif // LINK LAYER DETECTION
}

void
AODV::handle_link_failure(nsaddr_t id) {
aodv_rt_entry *rt, *rtn;
Packet *rerr = Packet::alloc();
struct hdr_aodv_error *re = HDR_AODV_ERROR(rerr);

 re->DestCount = 0;
 for(rt = rtable.head(); rt; rt = rtn) {  // for each rt entry
   rtn = rt->rt_link.le_next; 
   if ((rt->rt_hops != INFINITY2) && (rt->rt_nexthop == id) ) {
     assert (rt->rt_flags == RTF_UP);
     assert((rt->rt_seqno%2) == 0);
     rt->rt_seqno++;
     re->unreachable_dst[re->DestCount] = rt->rt_dst;
     re->unreachable_dst_seqno[re->DestCount] = rt->rt_seqno;
#ifdef DEBUG
     fprintf(stderr, "%s(%f): %d\t(%d\t%u\t%d)\n", __FUNCTION__, CURRENT_TIME,
		     index, re->unreachable_dst[re->DestCount],
		     re->unreachable_dst_seqno[re->DestCount], rt->rt_nexthop);
#endif // DEBUG
     re->DestCount += 1;
     rt_down(rt);
   }
   // remove the lost neighbor from all the precursor lists
   rt->pc_delete(id);
 }   

 if (re->DestCount > 0) {
#ifdef DEBUG
   fprintf(stderr, "%s(%f): %d\tsending RERR...\n", __FUNCTION__, CURRENT_TIME, index);
#endif // DEBUG
   sendError(rerr, false);
 }
 else {
   Packet::free(rerr);
 }
}

void
AODV::local_rt_repair(aodv_rt_entry *rt, Packet *p) {
#ifdef DEBUG
  fprintf(stderr,"%s: Dst - %d\n", __FUNCTION__, rt->rt_dst); 
#endif  
  // Buffer the packet 
  rqueue.enque(p);

  // mark the route as under repair 
  rt->rt_flags = RTF_IN_REPAIR;

  sendRequest(rt->rt_dst);

  // set up a timer interrupt
  Scheduler::instance().schedule(&lrtimer, p->copy(), rt->rt_req_timeout);
}

void
AODV::rt_update(aodv_rt_entry *rt, u_int32_t seqnum, u_int16_t metric,
	       	nsaddr_t nexthop, double expire_time) {

     rt->rt_seqno = seqnum;
     rt->rt_hops = metric;
     rt->rt_flags = RTF_UP;
     rt->rt_nexthop = nexthop;
     rt->rt_expire = expire_time;
}

void
AODV::rt_down(aodv_rt_entry *rt) {
  /*
   *  Make sure that you don't "down" a route more than once.
   */

  if(rt->rt_flags == RTF_DOWN) {
    return;
  }

  // assert (rt->rt_seqno%2); // is the seqno odd?
  rt->rt_last_hop_count = rt->rt_hops;
  rt->rt_hops = INFINITY2;
  rt->rt_flags = RTF_DOWN;
  rt->rt_nexthop = 0;
  rt->rt_expire = 0;

} /* rt_down function */

/*
  Route Handling Functions
*/

void
AODV::rt_resolve(Packet *p) {
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
aodv_rt_entry *rt;

 /*
  *  Set the transmit failure callback.  That
  *  won't change.
  */
 ch->xmit_failure_ = aodv_rt_failed_callback;
 ch->xmit_failure_data_ = (void*) this;
	rt = rtable.rt_lookup(ih->daddr());
 if(rt == 0) {
	  rt = rtable.rt_add(ih->daddr());
 }

 /*
  * If the route is up, forward the packet 
  */
	
 if(rt->rt_flags == RTF_UP) {
   assert(rt->rt_hops != INFINITY2);
   forward(rt, p, NO_DELAY);
 }
 /*
  *  if I am the source of the packet, then do a Route Request.
  */
	else if(ih->saddr() == index) {
   rqueue.enque(p);
   sendRequest(rt->rt_dst);
 }
 /*
  *	A local repair is in progress. Buffer the packet. 
  */
 else if (rt->rt_flags == RTF_IN_REPAIR) {
   rqueue.enque(p);
 }

 /*
  * I am trying to forward a packet for someone else to which
  * I don't have a route.
  */
 else {
 Packet *rerr = Packet::alloc();
 struct hdr_aodv_error *re = HDR_AODV_ERROR(rerr);
 /* 
  * For now, drop the packet and send error upstream.
  * Now the route errors are broadcast to upstream
  * neighbors - Mahesh 09/11/99
  */	
 
   assert (rt->rt_flags == RTF_DOWN);
   re->DestCount = 0;
   re->unreachable_dst[re->DestCount] = rt->rt_dst;
   re->unreachable_dst_seqno[re->DestCount] = rt->rt_seqno;
   re->DestCount += 1;
#ifdef DEBUG
   fprintf(stderr, "%s: sending RERR...\n", __FUNCTION__);
#endif
   sendError(rerr, false);

   drop(p, DROP_RTR_NO_ROUTE);
 }

}

void
AODV::rt_purge() {
aodv_rt_entry *rt, *rtn;
double now = CURRENT_TIME;
double delay = 0.0;
Packet *p;

 for(rt = rtable.head(); rt; rt = rtn) {  // for each rt entry
   rtn = rt->rt_link.le_next;
   if ((rt->rt_flags == RTF_UP) && (rt->rt_expire < now)) {
   // if a valid route has expired, purge all packets from 
   // send buffer and invalidate the route.                    
	assert(rt->rt_hops != INFINITY2);
     while((p = rqueue.deque(rt->rt_dst))) {
#ifdef DEBUG
       fprintf(stderr, "%s: calling drop()\n",
                       __FUNCTION__);
#endif // DEBUG
       drop(p, DROP_RTR_NO_ROUTE);
     }
     rt->rt_seqno++;
     assert (rt->rt_seqno%2);
     rt_down(rt);
   }
   else if (rt->rt_flags == RTF_UP) {
   // If the route is not expired,
   // and there are packets in the sendbuffer waiting,
   // forward them. This should not be needed, but this extra 
   // check does no harm.
     assert(rt->rt_hops != INFINITY2);
     while((p = rqueue.deque(rt->rt_dst))) {
       forward (rt, p, delay);
       delay += ARP_DELAY;
     }
   } 
   else if (rqueue.find(rt->rt_dst))
   // If the route is down and 
   // if there is a packet for this destination waiting in
   // the sendbuffer, then send out route request. sendRequest
   // will check whether it is time to really send out request
   // or not.
   // This may not be crucial to do it here, as each generated 
   // packet will do a sendRequest anyway.

     sendRequest(rt->rt_dst); 
   }

}

/*
  Packet Reception Routines
*/

void
AODV::recv(Packet *p, Handler*) {
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);

 assert(initialized());
 //assert(p->incoming == 0);
 // XXXXX NOTE: use of incoming flag has been depracated; In order to track direction of pkt flow, direction_ in hdr_cmn is used instead. see packet.h for details.

 if(ch->ptype() == PT_AODV) {
   ih->ttl_ -= 1;
   recvAODV(p);
   return;
 }


 /*
  *  Must be a packet I'm originating...
  */
if((ih->saddr() == index) && (ch->num_forwards() == 0)) {
 /*
  * Add the IP Header.  
  * TCP adds the IP header too, so to avoid setting it twice, we check if
  * this packet is not a TCP or ACK segment.
  */

#ifdef KIZON
  if (ch->ptype() != PT_TCP && ch->ptype() != PT_ACK) {
#endif //KIZON

    ch->size() += IP_HDR_LEN;

#ifdef KIZON
  }
#endif //KIZON

   // Added by Parag Dadhania && John Novatnack to handle broadcasting
  if ( (u_int32_t)ih->daddr() != IP_BROADCAST) 

#ifdef KIZON
{
#endif //KIZON

    ih->ttl_ = NETWORK_DIAMETER;
  }
#ifdef KIZON
}
#endif //KIZON
 /*
  *  I received a packet that I sent.  Probably
  *  a routing loop.
  */
else if(ih->saddr() == index) {
   drop(p, DROP_RTR_ROUTE_LOOP);
   return;
 }
 /*
  *  Packet I'm forwarding...
  */
 else {
 /*
  *  Check the TTL.  If it is zero, then discard.
  */
   if(--ih->ttl_ == 0) {
     drop(p, DROP_RTR_TTL);
     return;
   }
 }
// Added by Parag Dadhania && John Novatnack to handle broadcasting
 if ( (u_int32_t)ih->daddr() != IP_BROADCAST)
   rt_resolve(p);
 else
   forward((aodv_rt_entry*) 0, p, NO_DELAY);
}


void
AODV::recvAODV(Packet *p) {
 struct hdr_aodv *ah = HDR_AODV(p);
#ifdef PROP
 struct hdr_ip *ih = HDR_IP(p);
#endif //PROP

 assert(HDR_IP (p)->sport() == RT_PORT);
 assert(HDR_IP (p)->dport() == RT_PORT);

 /*
  * Incoming Packets.
  */
 switch(ah->ah_type) {

 case AODVTYPE_RREQ:
   recvRequest(p);
   break;

 case AODVTYPE_RREP:
   recvReply(p);
   break;

 case AODVTYPE_RERR:
   recvError(p);
   break;

 case AODVTYPE_HELLO:
   recvHello(p);
   break;
        
 default:
   fprintf(stderr, "Invalid AODV type (%x)\n", ah->ah_type);
   exit(1);
 }

}


void
AODV::recvRequest(Packet *p) {
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);
aodv_rt_entry *rt;
#ifdef PROP
AODV_Neighbor *nb; 
#endif //PROP

  /*
   * Drop if:
   *      - I'm the source
   *      - I recently heard this request.
   */


  if(rq->rq_src == index) {
#ifdef DEBUG
    fprintf(stderr, "%s: got my own REQUEST\n", __FUNCTION__);
#endif // DEBUG
    Packet::free(p);
    return;
  } 

 if (id_lookup(rq->rq_src, rq->rq_bcast_id)) { //すでにRREQパケットを受け取ったことがある場合は、True(重複受信処理)を行う

#ifdef DEBUG
   fprintf(stderr, "%s: discarding request\n", __FUNCTION__);
#endif // DEBUG

#ifdef PROP
if (rq->rq_dst != index ) { //自身がRREQの宛先ではない場合
#endif //PROP
 
   Packet::free(p);
   return;
 }

//tuika------------------------------//
#ifdef PROP                                       //長田さん追加　経路選択
else{ //自身が宛先端末である場合の処理

if(nb = nb_lookup(rq->rq_prev_id)){
my_time = nb->st_time;
if(my_time == 0.0){
   my_time = 1000.0; 
   }    
}

rq->rq_time[rq->rq_hop_count-1] = my_time;
/*
rq->rq_min_time = rq->rq_time[0]; 
for(int i = 1; i < rq->rq_hop_count; i++){
		rq->rq_min_time = rq->rq_min_time + rq->rq_time[i];
}
rq->rq_min_time = rq->rq_min_time/rq->rq_hop_count;
*/
 
rq->rq_min_time = rq->rq_time[0]; 
for(int i = 0; i < rq->rq_hop_count; i++){
//printf("2kaime-rqtime:%.5f\n",rq->rq_time[i]);
printf("route_hop_cnt:%d, rq_time:%.20f\n", rq->rq_hop_count, rq->rq_time[i]);
	if(rq->rq_min_time > rq->rq_time[i]){
		rq->rq_min_time = rq->rq_time[i];
	}
}
printf("rq_min_time:%.20f\n", rq->rq_min_time);

v_all_time.push_back(rq->rq_min_time);      // v_all_time の初期化必要
int mid_cnt_low = int(v_all_time.size()/2); // if 13 -> 6, 14 -> 7
int mid_cnt_high = int(v_all_time.size()/2)+1;  // if 13 -> 7, 14 -> 8
printf("route_cadidate_size:%d\n",v_all_time.size());
//printf("low_cnt:%d\n",mid_cnt_low);
//printf("high_cnt:%d\n",mid_cnt_high);
std::sort(v_all_time.begin(), v_all_time.end(), std::greater<double>());
std::vector<double>::iterator it = v_all_time.begin();
//printf("it:%.1f\n",*it);
int loop_cnt = 1; 

for(it = v_all_time.begin(); it != v_all_time.end(); ++it) {
    if (loop_cnt == mid_cnt_low) {
        dst_mid_time_low[rq->rq_src][rq->rq_bcast_id] = *it; 
//		printf("low:%.1f\n",dst_mid_time_low[rq->rq_src][rq->rq_bcast_id]); 
    }    
    if (loop_cnt == mid_cnt_high) {
        dst_mid_time_high[rq->rq_src][rq->rq_bcast_id] = *it; 
//		printf("hig:%.1f\n",dst_mid_time_high[rq->rq_src][rq->rq_bcast_id]); 
    }    
    loop_cnt++;
}

int aodv_loop_cnt = 0;

for(it = v_all_time.begin(); it != v_all_time.end(); ++it) {

printf("route_cadidate:%dth:current-time:%.20f\nkouho-it:%.20f\n", aodv_loop_cnt, CURRENT_TIME, *it);

    if (*it == one_time) {
        aodv_no = aodv_loop_cnt;
    }
    aodv_loop_cnt++;

}
printf("aodv-order:%d\n",aodv_no);
printf("cond1:rq_min_time:%.20f, dst_min_time:%.20f\n", rq->rq_min_time, dst_min_time[rq->rq_src][rq->rq_bcast_id]);

//		printf("aaa-mintime:%.5f\n",dst_min_time[rq->rq_src][rq->rq_bcast_id]);
//		printf("bbb-mintime:%.5f\n",rq->rq_min_time);
	if( (dst_timeout[rq->rq_src][rq->rq_bcast_id] > CURRENT_TIME) && 
	(rq->rq_min_time > dst_min_time[rq->rq_src][rq->rq_bcast_id]) ) { 

//	if(dst_mid_time_low[rq->rq_src][rq->rq_bcast_id] < rq->rq_min_time &&
//		 rq->rq_min_time < dst_mid_time_high[rq->rq_src][rq->rq_bcast_id])

//		if(dst_hop_count[rq->rq_src][rq->rq_bcast_id] > 0
//		&& dst_hop_count[rq->rq_src][rq->rq_bcast_id] +1  >= rq->rq_hop_count) {

		printf("cond2:dst_hop_count:%d, rq_hop_count:%d\n", dst_hop_count[rq->rq_src][rq->rq_bcast_id], rq->rq_hop_count);
		printf("cond3:rq_min_time:%.20f\n", rq->rq_min_time);
		if(dst_hop_count[rq->rq_src][rq->rq_bcast_id] == rq->rq_hop_count && rq->rq_min_time < 10000.0) {  //RREQ選択でホップ数が同じか判断して同じなら選択に入る
		                                                                                                   //長田さんは時間もトリガーだが南雲はさらしにすればいい？
   
	dst_min_time[rq->rq_src][rq->rq_bcast_id] = rq->rq_min_time; //現在までの経路最小電波強度値の最大値を更新する.
		printf("= kosin = node_no:%d, rq_hop:%d, min_time:%.20f\n", index, rq->rq_hop_count, dst_min_time[rq->rq_src][rq->rq_bcast_id]);
		printf("============================\n");
	ListScheduler::instance().cancel( dst_pkt_ptr[rq->rq_src][rq->rq_bcast_id] ); //すでにスケジューリング済みのRREPをキャンセルする　　南雲追加するべきここから

//探索元へのリバース経路を、新しい経路へとアップデートする
	aodv_rt_entry *rt0; // rt0 is the reverse route
	rt0 = rtable.rt_lookup(rq->rq_src);
	if(rt0 == 0) { /* if not in the route table */
		rt0 = rtable.rt_add(rq->rq_src);
	}
	rt0->rt_expire = max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE));
	rt_update(rt0, rq->rq_src_seqno, rq->rq_hop_count, ih->saddr(), max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE)) );

	if (rt0->rt_req_timeout > 0.0) {
		rt0->rt_req_cnt = 0;
		rt0->rt_req_timeout = 0.0;
		rt0->rt_req_last_ttl = rq->rq_hop_count;
		rt0->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
	}

    sendReply(rq->rq_src,     // IP Destination
             1,                    // Hop Count
             index,                // Dest IP Address
             seqno,                // Dest Sequence Num
             MY_ROUTE_TIMEOUT,     // Lifetime
             rq->rq_timestamp,     // timestamp
             rq->rq_src,           // index
             rq->rq_bcast_id,      // bcast_id
             dst_timeout[rq->rq_src][rq->rq_bcast_id] - CURRENT_TIME); //RREPの送信をリスケジューリングする(送信が開始される時刻dst_timeoutから、現在時間をさし引いた時間に送信される)
                                                                       //ここは時間で選んでいるが南雲さらし端末数を参考にする
		Packet::free(p);
//check_flg = 0;                  // check_flg の初期化必要
		return;
		}
	}
		else{ //はじめのRREQを受信してからすでにTwait秒以上経過している場合
			Packet::free(p);
			return;
		}	
	}
}
南雲ここまで追加するべき
#endif //PROP
//------------------------------//

 /*
  * Cache the broadcast ID
  */
 id_insert(rq->rq_src, rq->rq_bcast_id);


 /* 
  * We are either going to forward the REQUEST or generate a
  * REPLY. Before we do anything, we make sure that the REVERSE
  * route is in the route table.
  */
 aodv_rt_entry *rt0; // rt0 is the reverse route 
   
   rt0 = rtable.rt_lookup(rq->rq_src);
   if(rt0 == 0) { /* if not in the route table */
   // create an entry for the reverse route.
     rt0 = rtable.rt_add(rq->rq_src);
   }
  
   rt0->rt_expire = max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE));

   if ( (rq->rq_src_seqno > rt0->rt_seqno ) ||
    	((rq->rq_src_seqno == rt0->rt_seqno) && 
	 (rq->rq_hop_count < rt0->rt_hops)) ) {
   // If we have a fresher seq no. or lesser #hops for the 
   // same seq no., update the rt entry. Else don't bother.
rt_update(rt0, rq->rq_src_seqno, rq->rq_hop_count, ih->saddr(),
     	       max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE)) );
     if (rt0->rt_req_timeout > 0.0) {
     // Reset the soft state and 
     // Set expiry time to CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT
     // This is because route is used in the forward direction,
     // but only sources get benefited by this change
       rt0->rt_req_cnt = 0;
       rt0->rt_req_timeout = 0.0; 
       rt0->rt_req_last_ttl = rq->rq_hop_count;
       rt0->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
     }

     /* Find out whether any buffered packet can benefit from the 
      * reverse route.
      * May need some change in the following code - Mahesh 09/11/99
      */
     assert (rt0->rt_flags == RTF_UP);
     Packet *buffered_pkt;
     while ((buffered_pkt = rqueue.deque(rt0->rt_dst))) {
       if (rt0 && (rt0->rt_flags == RTF_UP)) {
	assert(rt0->rt_hops != INFINITY2);
         forward(rt0, buffered_pkt, NO_DELAY);
       }
     }
   } 
   // End for putting reverse route in rt table


 /*
  * We have taken care of the reverse route stuff.
  * Now see whether we can send a route reply. 
  */

 rt = rtable.rt_lookup(rq->rq_dst);

 // First check if I am the destination ..

 if(rq->rq_dst == index) { //自身がRREQの宛先である処理(初回受信時)

//tuika------------------------------//
#ifdef PROP

v_all_time.clear();      // v_all_time の初期化必要
dst_hop_count[rq->rq_src][rq->rq_bcast_id] = rq->rq_hop_count; //初回RREQのホップ数を記録
printf("==== 1st:current-time:%.5f, node_no:%d, hop_count:%.1f\n",CURRENT_TIME,index,dst_hop_count[rq->rq_src][rq->rq_bcast_id]);

if(nb = nb_lookup(rq->rq_prev_id)){
    my_time = nb->st_time;      // 前中継者ノードから受信取得する.
    if(my_time == 0.0){
        my_time = 1000.0;
    }
}
rq->rq_time[rq->rq_hop_count-1] = my_time;
//printf("mytime:%.1f-rqtime:%.1f\n",my_time,rq->rq_time[rq->rq_hop_count-1]);
rq->rq_min_time = rq->rq_time[0];  
	for(int i = 0; i < rq->rq_hop_count; i++){
//printf("shokai-rqtime:%.5f\n",rq->rq_time[i]);
printf("1st:aodv:route_hop_cnt:%d, rq_time:%.20f\n", rq->rq_hop_count, rq->rq_time[i]);
		if(rq->rq_min_time > rq->rq_time[i]){
			rq->rq_min_time = rq->rq_time[i];
		}
	}
	dst_min_time[rq->rq_src][rq->rq_bcast_id] = rq->rq_min_time; //現在までの経路最小電波強度値の最大値を更新する.

one_time = rq->rq_min_time;
printf("1st:aodv:rq_min_time:%.20f\n", one_time);
v_all_time.push_back(rq->rq_min_time);      // v_all_time の初期化必要
	printf("= 1st-fix = node_no:%d, rq_hop:%d, rq_dst%d, min_time:%.20f\n", index, rq->rq_hop_count, rq->rq_dst, dst_min_time[rq->rq_src][rq->rq_bcast_id]);

//dst_timeout[rq->rq_src][rq->rq_bcast_id] = 0.0;
dst_timeout[rq->rq_src][rq->rq_bcast_id] = Twait + CURRENT_TIME;   // タイムアウトする時刻の算出.
//printf("RREQwaittime:%.5f\n",Twait);   // タイムアウトする時刻の算出.
#endif //PROP
//------------------------------//

#ifdef DEBUG
   fprintf(stderr, "%d - %s: destination sending reply\n",
                   index, __FUNCTION__);
#endif // DEBUG

               
   // Just to be safe, I use the max. Somebody may have
   // incremented the dst seqno.
   seqno = max(seqno, rq->rq_dst_seqno)+1;
   if (seqno%2) seqno++;

   sendReply(rq->rq_src,           // IP Destination
             1,                    // Hop Count
             index,                // Dest IP Address
             seqno,                // Dest Sequence Num
             MY_ROUTE_TIMEOUT,     // Lifetime
             rq->rq_timestamp      // timestamp
//#ifdef PROP
           , rq->rq_src,           // index
             rq->rq_bcast_id,      // bcast_id
             Twait                 // Twait(50[ms])後に送信される.
//#endif //PROP
			);
 
   Packet::free(p);
 }


// 提案では中間リプレイの禁止するためコメントアウトする(917-958行目まで)
#ifdef REPLY 
 // I am not the destination, but I may have a fresh enough route.

 else if (rt && (rt->rt_hops != INFINITY2) && 
	  	(rt->rt_seqno >= rq->rq_dst_seqno) ) {

   //assert (rt->rt_flags == RTF_UP);
   assert(rq->rq_dst == rt->rt_dst);
   //assert ((rt->rt_seqno%2) == 0);	// is the seqno even?
   sendReply(rq->rq_src,
             rt->rt_hops + 1,
             rq->rq_dst,
             rt->rt_seqno,
	     (u_int32_t) (rt->rt_expire - CURRENT_TIME),
	     //             rt->rt_expire - CURRENT_TIME,
             rq->rq_timestamp);
   // Insert nexthops to RREQ source and RREQ destination in the
   // precursor lists of destination and source respectively
   rt->pc_insert(rt0->rt_nexthop); // nexthop to RREQ source
   rt0->pc_insert(rt->rt_nexthop); // nexthop to RREQ destination

/*
//#ifdef RREQ_GRAT_RREP  

   sendReply(rq->rq_dst,
             rq->rq_hop_count,
             rq->rq_src,
             rq->rq_src_seqno,
	     (u_int32_t) (rt->rt_expire - CURRENT_TIME),
	     //             rt->rt_expire - CURRENT_TIME,
             rq->rq_timestamp);
//#endif //RREQ_GRAT_RREP  
*/
   
// TODO: send grat RREP to dst if G flag set in RREQ using rq->rq_src_seqno, rq->rq_hop_counT
   
// DONE: Included gratuitous replies to be sent as per IETF aodv draft specification. As of now, G flag has not been dynamically used and is always set or reset in aodv-packet.h --- Anant Utgikar, 09/16/02.

	Packet::free(p);
 }

#endif //REPLY

 /*
  * Can't reply. So forward the  Route Request
  */
 else {
   ih->saddr() = index;
   ih->daddr() = IP_BROADCAST;
   rq->rq_hop_count += 1;
   // Maximum sequence number seen en route
   if (rt) rq->rq_dst_seqno = max(rt->rt_seqno, rq->rq_dst_seqno);
   forward((aodv_rt_entry*) 0, p, DELAY);
 }

}


void
AODV::recvReply(Packet *p) {
//struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
aodv_rt_entry *rt;
char suppress_reply = 0;
double delay = 0.0;
	
#ifdef DEBUG
 fprintf(stderr, "%d - %s: received a REPLY\n", index, __FUNCTION__);
#endif // DEBUG


 /*
  *  Got a reply. So reset the "soft state" maintained for 
  *  route requests in the request table. We don't really have
  *  have a separate request table. It is just a part of the
  *  routing table itself. 
  */
 // Note that rp_dst is the dest of the data packets, not the
 // the dest of the reply, which is the src of the data packets.

 rt = rtable.rt_lookup(rp->rp_dst);
        
 /*
  *  If I don't have a rt entry to this host... adding
  */
 if(rt == 0) {
   rt = rtable.rt_add(rp->rp_dst);
 }

 /*
  * Add a forward route table entry... here I am following 
  * Perkins-Royer AODV paper almost literally - SRD 5/99
  */

 if ( (rt->rt_seqno < rp->rp_dst_seqno) ||   // newer route 
      ((rt->rt_seqno == rp->rp_dst_seqno) &&  
       (rt->rt_hops > rp->rp_hop_count)) ) { // shorter or better route
	
  // Update the rt entry 
  rt_update(rt, rp->rp_dst_seqno, rp->rp_hop_count,
		rp->rp_src, CURRENT_TIME + rp->rp_lifetime);

  // reset the soft state
  rt->rt_req_cnt = 0;
  rt->rt_req_timeout = 0.0; 
  rt->rt_req_last_ttl = rp->rp_hop_count;
  
if (ih->daddr() == index) { // If I am the original source
  // Update the route discovery latency statistics
  // rp->rp_timestamp is the time of request origination
		
    rt->rt_disc_latency[(unsigned char)rt->hist_indx] = (CURRENT_TIME - rp->rp_timestamp)
                                         / (double) rp->rp_hop_count;
    // increment indx for next time
    rt->hist_indx = (rt->hist_indx + 1) % MAX_HISTORY;
  }	

  /*
   * Send all packets queued in the sendbuffer destined for
   * this destination. 
   * XXX - observe the "second" use of p.
   */
  Packet *buf_pkt;
  while((buf_pkt = rqueue.deque(rt->rt_dst))) {
    if(rt->rt_hops != INFINITY2) {
          assert (rt->rt_flags == RTF_UP);
    // Delay them a little to help ARP. Otherwise ARP 
    // may drop packets. -SRD 5/23/99
      forward(rt, buf_pkt, delay);
      delay += ARP_DELAY;
    }
  }
 }
 else {
  suppress_reply = 1;
 }

 /*
  * If reply is for me, discard it.
  */

if(ih->daddr() == index || suppress_reply) {
   Packet::free(p);
 }
 /*
  * Otherwise, forward the Route Reply.
  */
 else {
 // Find the rt entry
aodv_rt_entry *rt0 = rtable.rt_lookup(ih->daddr());
   // If the rt is up, forward
   if(rt0 && (rt0->rt_hops != INFINITY2)) {
        assert (rt0->rt_flags == RTF_UP);
     rp->rp_hop_count += 1;
     rp->rp_src = index;
     forward(rt0, p, NO_DELAY);
     // Insert the nexthop towards the RREQ source to 
     // the precursor list of the RREQ destination
     rt->pc_insert(rt0->rt_nexthop); // nexthop to RREQ source
     
   }
   else {
   // I don't know how to forward .. drop the reply. 
#ifdef DEBUG
     fprintf(stderr, "%s: dropping Route Reply\n", __FUNCTION__);
#endif // DEBUG
     drop(p, DROP_RTR_NO_ROUTE);
   }
 }
}


void
AODV::recvError(Packet *p) {
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_error *re = HDR_AODV_ERROR(p);
aodv_rt_entry *rt;
u_int8_t i;
Packet *rerr = Packet::alloc();
struct hdr_aodv_error *nre = HDR_AODV_ERROR(rerr);

 nre->DestCount = 0;

 for (i=0; i<re->DestCount; i++) {
 // For each unreachable destination
   rt = rtable.rt_lookup(re->unreachable_dst[i]);
   if ( rt && (rt->rt_hops != INFINITY2) &&
	(rt->rt_nexthop == ih->saddr()) &&
     	(rt->rt_seqno <= re->unreachable_dst_seqno[i]) ) {
	assert(rt->rt_flags == RTF_UP);
	assert((rt->rt_seqno%2) == 0); // is the seqno even?
#ifdef DEBUG
     fprintf(stderr, "%s(%f): %d\t(%d\t%u\t%d)\t(%d\t%u\t%d)\n", __FUNCTION__,CURRENT_TIME,
		     index, rt->rt_dst, rt->rt_seqno, rt->rt_nexthop,
		     re->unreachable_dst[i],re->unreachable_dst_seqno[i],
	             ih->saddr());
#endif // DEBUG
     	rt->rt_seqno = re->unreachable_dst_seqno[i];
     	rt_down(rt);

   // Not sure whether this is the right thing to do
   Packet *pkt;
	while((pkt = ifqueue->filter(ih->saddr()))) {
        	drop(pkt, DROP_RTR_MAC_CALLBACK);
     	}

     // if precursor list non-empty add to RERR and delete the precursor list
     	if (!rt->pc_empty()) {
     		nre->unreachable_dst[nre->DestCount] = rt->rt_dst;
     		nre->unreachable_dst_seqno[nre->DestCount] = rt->rt_seqno;
     		nre->DestCount += 1;
		rt->pc_delete();
     	}
   }
 } 

 if (nre->DestCount > 0) {
#ifdef DEBUG
   fprintf(stderr, "%s(%f): %d\t sending RERR...\n", __FUNCTION__, CURRENT_TIME, index);
#endif // DEBUG
   sendError(rerr);
 }
 else {
   Packet::free(rerr);
 }

 Packet::free(p);
}


/*
   Packet Transmission Routines
*/

void
AODV::forward(aodv_rt_entry *rt, Packet *p, double delay) {
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);

//tuika------------------------------//
#ifdef PROP
struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p); //PROPmisu ポインタrqを通してRREQヘッダにアクセスできるようにする.
AODV_Neighbor *nb; //PROPmisu
//printf("111:%d\n",rq->rq_prev_id);
//printf("222:%d\n",rq->rq_dst);
//printf("333:%d\n",index);
//printf("444:%d\n",nb);
//printf("444:%f\n",nb);
//printf("555:%d\n",nb_lookup(rq->rq_prev_id));


if(nb = nb_lookup(rq->rq_prev_id)){
    my_time = nb->st_time;
    if(my_time == 0.0){
        my_time = 1000.0;
    }
}
#endif //PROP
//------------------------------//

 if(ih->ttl_ == 0) {

#ifdef DEBUG
  fprintf(stderr, "%s: calling drop()\n", __PRETTY_FUNCTION__);
#endif // DEBUG
 
  drop(p, DROP_RTR_TTL);
  return;
 }

 if ((( ch->ptype() != PT_AODV && ch->direction() == hdr_cmn::UP ) &&
	((u_int32_t)ih->daddr() == IP_BROADCAST))
		|| (ih->daddr() == here_.addr_)) {
	dmux_->recv(p,0);
	return;
 }

 if (rt) {
   assert(rt->rt_flags == RTF_UP);
   rt->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
   ch->next_hop_ = rt->rt_nexthop;
   ch->addr_type() = NS_AF_INET;
   ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
 }
 else { // if it is a broadcast packet
   // assert(ch->ptype() == PT_AODV); // maybe a diff pkt type like gaf
   assert(ih->daddr() == (nsaddr_t) IP_BROADCAST);
   ch->addr_type() = NS_AF_NONE;
   ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction
 }

if (ih->daddr() == (nsaddr_t) IP_BROADCAST) {
 // If it is a broadcast packet
   assert(rt == 0);

#ifdef KIZON
   if (ch->ptype() == PT_AODV) {
#endif //KIZON

     /*
      *  Jitter the sending of AODV broadcast packets by 10ms
      */

#ifdef PROP
rq->rq_time[rq->rq_hop_count-2] = my_time;
rq->rq_prev_id = index;
#endif //PROP

     Scheduler::instance().schedule(target_, p,
      				   0.01 * Random::uniform());


#ifdef KIZON
   } else {
     Scheduler::instance().schedule(target_, p, 0.);  // No jitter
   }
#endif //KIZON

 }
 else { // Not a broadcast packet 
   if(delay > 0.0) {
     Scheduler::instance().schedule(target_, p, delay);
   }
   else {
   // Not a broadcast packet, no delay, send immediately
     Scheduler::instance().schedule(target_, p, 0.);
   }
 }

}


void
AODV::sendRequest(nsaddr_t dst) {
// Allocate a RREQ packet 
Packet *p = Packet::alloc();
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);
aodv_rt_entry *rt = rtable.rt_lookup(dst);

 assert(rt);

 /*
  *  Rate limit sending of Route Requests. We are very conservative
  *  about sending out route requests. 
  */

 if (rt->rt_flags == RTF_UP) {
   assert(rt->rt_hops != INFINITY2);
   Packet::free((Packet *)p);
   return;
 }

 if (rt->rt_req_timeout > CURRENT_TIME) {
   Packet::free((Packet *)p);
   return;
 }

 // rt_req_cnt is the no. of times we did network-wide broadcast
 // RREQ_RETRIES is the maximum number we will allow before 
 // going to a long timeout.

 if (rt->rt_req_cnt > RREQ_RETRIES) {
   rt->rt_req_timeout = CURRENT_TIME + MAX_RREQ_TIMEOUT;
   rt->rt_req_cnt = 0;
 Packet *buf_pkt;
   while ((buf_pkt = rqueue.deque(rt->rt_dst))) {
       drop(buf_pkt, DROP_RTR_NO_ROUTE);
   }
   Packet::free((Packet *)p);
   return;
 }

#ifdef DEBUG
   fprintf(stderr, "(%2d) - %2d sending Route Request, dst: %d\n",
                    ++route_request, index, rt->rt_dst);
#endif // DEBUG
//qwer   fprintf(stderr, "- %2d sending Route Request, dst: %d\n",
//                    index, rt->rt_dst);

 // Determine the TTL to be used this time. 
 // Dynamic TTL evaluation - SRD

 rt->rt_req_last_ttl = max(rt->rt_req_last_ttl,rt->rt_last_hop_count);

 if (0 == rt->rt_req_last_ttl) {
 // first time query broadcast
   ih->ttl_ = TTL_START;
 }
 else {
 // Expanding ring search.
   if (rt->rt_req_last_ttl < TTL_THRESHOLD)
     ih->ttl_ = rt->rt_req_last_ttl + TTL_INCREMENT;
   else {
   // network-wide broadcast
     ih->ttl_ = NETWORK_DIAMETER;
     rt->rt_req_cnt += 1;
   }
 }

 // remember the TTL used  for the next time
 rt->rt_req_last_ttl = ih->ttl_;

 // PerHopTime is the roundtrip time per hop for route requests.
 // The factor 2.0 is just to be safe .. SRD 5/22/99
 // Also note that we are making timeouts to be larger if we have 
 // done network wide broadcast before. 

 rt->rt_req_timeout = 2.0 * (double) ih->ttl_ * PerHopTime(rt); 
 if (rt->rt_req_cnt > 0)
   rt->rt_req_timeout *= rt->rt_req_cnt;
 rt->rt_req_timeout += CURRENT_TIME;

 // Don't let the timeout to be too large, however .. SRD 6/8/99
 if (rt->rt_req_timeout > CURRENT_TIME + MAX_RREQ_TIMEOUT)
   rt->rt_req_timeout = CURRENT_TIME + MAX_RREQ_TIMEOUT;
 rt->rt_expire = 0;

#ifdef DEBUG
 fprintf(stderr, "(%2d) - %2d sending Route Request, dst: %d, tout %f ms\n",
	         ++route_request, 
		 index, rt->rt_dst, 
		 rt->rt_req_timeout - CURRENT_TIME);
#endif	// DEBUG
	

 // Fill out the RREQ packet 
 // ch->uid() = 0;
 ch->ptype() = PT_AODV;
 ch->size() = IP_HDR_LEN + rq->size();
 ch->iface() = -2;
 ch->error() = 0;
 ch->addr_type() = NS_AF_NONE;
 ch->prev_hop_ = index;          // AODV hack

 ih->saddr() = index;
 ih->daddr() = IP_BROADCAST;
 ih->sport() = RT_PORT;
 ih->dport() = RT_PORT;

 // Fill up some more fields. 
 rq->rq_type = AODVTYPE_RREQ;
 rq->rq_hop_count = 1;
 rq->rq_bcast_id = bid++;
 rq->rq_dst = dst;
 rq->rq_dst_seqno = (rt ? rt->rt_seqno : 0);
 rq->rq_src = index;
 seqno += 2;
 assert ((seqno%2) == 0);
 rq->rq_src_seqno = seqno;
 rq->rq_timestamp = CURRENT_TIME;

#ifdef PROP
rq->rq_prev_id = index;
 for(int i = 0; i < 100; i++){ //フィールドの初期化
     rq->rq_time[i] = 0.0;
    }
rq->rq_min_time = 0.0;  
#endif //PROP

#ifdef MISU
rtable.rt_dump(index);
#endif //MISU

 Scheduler::instance().schedule(target_, p, 0.);

}


//#ifdef KIZON
//void
//AODV::sendReply(nsaddr_t ipdst, u_int32_t hop_count, nsaddr_t rpdst, u_int32_t rpseq, u_int32_t lifetime, double timestamp) {
//#endif //KIZON

//#ifdef PROP
void
AODV::sendReply(nsaddr_t ipdst, u_int32_t hop_count, nsaddr_t rpdst,u_int32_t rpseq, u_int32_t lifetime, double timestamp, int rq_src, int rq_bcast_id, double dst_wait) {
//#endif //PROP

Packet *p = Packet::alloc();
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
aodv_rt_entry *rt = rtable.rt_lookup(ipdst);

#ifdef PROP
struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p); //PROP
dst_pkt_ptr[rq_src][rq_bcast_id] = p; //PROPmisu
#endif //PROP

#ifdef DEBUG
fprintf(stderr, "sending Reply from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG
 assert(rt);

 rp->rp_type = AODVTYPE_RREP;
 //rp->rp_flags = 0x00;
 rp->rp_hop_count = hop_count;
 rp->rp_dst = rpdst;
 rp->rp_dst_seqno = rpseq;
 rp->rp_src = index;
 rp->rp_lifetime = lifetime;
 rp->rp_timestamp = timestamp;
   
 // ch->uid() = 0;
 ch->ptype() = PT_AODV;
 ch->size() = IP_HDR_LEN + rp->size();
 ch->iface() = -2;
 ch->error() = 0;
 ch->addr_type() = NS_AF_INET;
 ch->next_hop_ = rt->rt_nexthop;
 ch->prev_hop_ = index;          // AODV hack
 ch->direction() = hdr_cmn::DOWN;

 ih->saddr() = index;
 ih->daddr() = ipdst;
 ih->sport() = RT_PORT;
 ih->dport() = RT_PORT;
 ih->ttl_ = NETWORK_DIAMETER;

#ifdef DST
 Scheduler::instance().schedule(target_, p, 0.05);
#endif //DST

#ifdef PROP
// Scheduler::instance().schedule(target_, p, dst_wait); //PROPmisu
// Scheduler::instance().schedule(target_, p, 0.05);
#endif //PROP

//printf("[%.5fsec][Node:%d][dst:%d][hop:%d]\n" ,CURRENT_TIME,index,rp->rp_dst,rp->rp_hop_count);
//printf("[%.5fsec]RREPをスケジューリングしました.\n",CURRENT_TIME);

#ifdef MISU
rtable.rt_dump(index);
#endif //MISU
}

void
AODV::sendError(Packet *p, bool jitter) {
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_error *re = HDR_AODV_ERROR(p);

printf("setudan\n");
printf("nu[time:%.5f] [id:%d] [s-moto:%d] \n",CURRENT_TIME, index, ih->saddr());

#ifdef ERROR
fprintf(stderr, "sending Error from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG

 re->re_type = AODVTYPE_RERR;
 //re->reserved[0] = 0x00; re->reserved[1] = 0x00;
 // DestCount and list of unreachable destinations are already filled

 // ch->uid() = 0;
 ch->ptype() = PT_AODV;
 ch->size() = IP_HDR_LEN + re->size();
 ch->iface() = -2;
 ch->error() = 0;
 ch->addr_type() = NS_AF_NONE;
 ch->next_hop_ = 0;
 ch->prev_hop_ = index;          // AODV hack
 ch->direction() = hdr_cmn::DOWN;       //important: change the packet's direction

 ih->saddr() = index;
 ih->daddr() = IP_BROADCAST;
 ih->sport() = RT_PORT;
 ih->dport() = RT_PORT;
 ih->ttl_ = 1;

 // Do we need any jitter? Yes
 if (jitter)
 	Scheduler::instance().schedule(target_, p, 0.01*Random::uniform());
 else
 	Scheduler::instance().schedule(target_, p, 0.0);

}


/*
   Neighbor Management Functions
*/

void
AODV::sendHello() {
Packet *p = Packet::alloc();
struct hdr_cmn *ch = HDR_CMN(p);
struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_reply *rh = HDR_AODV_REPLY(p);

#ifdef DEBUG
fprintf(stderr, "sending Hello from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG

 rh->rp_type = AODVTYPE_HELLO;
 //rh->rp_flags = 0x00;
 rh->rp_hop_count = 1;
 rh->rp_dst = index;
 rh->rp_dst_seqno = seqno;
 rh->rp_lifetime = (1 + ALLOWED_HELLO_LOSS) * HELLO_INTERVAL;

 // ch->uid() = 0;
 ch->ptype() = PT_AODV;
 ch->size() = IP_HDR_LEN + rh->size();
 ch->iface() = -2;
 ch->error() = 0;
 ch->addr_type() = NS_AF_NONE;
 ch->prev_hop_ = index;          // AODV hack

 ih->saddr() = index;
 ih->daddr() = IP_BROADCAST;
 ih->sport() = RT_PORT;
 ih->dport() = RT_PORT;
 ih->ttl_ = 1;

 Scheduler::instance().schedule(target_, p, 0.0);

#ifdef MISU
nb_dump(index);
#endif //MISU

}


void
AODV::recvHello(Packet *p) {
//struct hdr_ip *ih = HDR_IP(p);
struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
AODV_Neighbor *nb;

nb = nb_lookup(rp->rp_dst);
	if(nb == 0) {
//		nb_insert(rp->rp_dst); //KIZON
		nb_insert(rp->rp_dst,p); //PROP
	}
	else {
		nb->nb_expire = CURRENT_TIME + (1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);

//tuika--------------------//
#ifdef PROP
int ka;
double lambda = 300000000/914e+6;
double PI = 3.14159265359;
double kyori = sqrt((0.28183815 * 1 * 1 * lambda * lambda) / (1 * (p->txinfo_.RxPr))) / (4 * PI); 
distance = round(kyori);
		if(distance == 0.0 ){
			distance = 1;
		}

		if(nb->nb_distance[index][N] != 0){
			N++;
		}
		else{
			nb->nb_distance[index][N] = distance;
		}

double e = 1;
double x = 0, y = 0, sumXY = 0, sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, seppen = 0;
double cu_time = round(CURRENT_TIME);
double off[100];
double on[100];
double same[100];
int q, of1 = 0, on1 = 0, sa1 = 0;
nb->st_time = CURRENT_TIME + 10000; //何秒後に切断するか計算+現在時刻と付け加える

		for(q = 0; nb->nb_distance[index][q] != 0; q++){ //取得した距離の数をカウント
			if(nb->nb_distance[index][q] != 0 ){
               	int co = 1;
               	nb->app_count[index] = q + co;
			}

			if(nb->app_count[index] >= 2 ){ //カウント数が2以上（距離履歴が2つ以上の場合
//				printf("id:[%d]kaunt_2up\n",index);
				printf("realtime:[%.5f]\n",CURRENT_TIME);

				if(nb->nb_distance[index][q] > nb->nb_distance[index][q-1] ){ //離れる場合 
					of1 = 1;
					x = 0, y = 0, sumXY = 0, sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, seppen = 0;
					double off[100] = {0};
					for(int h = q; nb->nb_distance[index][h] > nb->nb_distance[index][h-1] && h >= 0; h--){
						off[of1-1] = nb->nb_distance[index][h];
//						printf("hana-st\n");
//						printf("[off:%d]\n",of1);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,off[0]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,off[1]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,off[2]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,off[3]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,off[4]);
 
                        x = of1;
                        y = off[of1-1];
                        sumXY += x * y;
                        sumX  += x;
                        sumY  += y;
                        sumX2 += x * x;
                        sumY2 += y * y;
//						printf("[of1:%d]\n",of1);
//						printf("[x:%.5f]\n",x);
//						printf("[y:%.5f]\n",y);
//						printf("[xy:%.5f]\n",sumXY);
//						printf("[sumx:%.5f]\n",sumX);
//						printf("[sumy:%.5f]\n",sumY);
//						printf("[x2:%.5f]\n",sumX2);
//						printf("[y2:%.5f]\n",sumY2);
						of1++;
					}
					if(of1 > 2){
//						printf("h-kinzi-start\n");
//						printf("[off:%d]\n",of1);
//						printf("[x:%.5f]\n",x);
//						printf("[y:%.5f]\n",y);
//						printf("[xy:%.5f]\n",sumXY);
//						printf("[sumx:%.5f]\n",sumX);
//						printf("[sumy:%.5f]\n",sumY);
//						printf("[x2:%.5f]\n",sumX2);
//						printf("[y2:%.5f]\n",sumY2);

//						printf("a%.5f\n",(x * sumXY));
//						printf("aa%.5f\n",(sumX * sumY));
//						printf("aaa%.5f\n",((x * sumXY) - (sumX * sumY)));
//						printf("b%.5f\n",(x * sumX2));
//						printf("bb%.5f\n",(sumX * sumX));
//						printf("bbb%.5f\n",((x * sumX2) - (sumX * sumX)));
//						printf("cc%.5f\n",(x * sumXY - sumX * sumY) / (x * sumX2 - sumX * sumX));
						nb->fa_kinzi[index] = (x * sumXY - sumX * sumY) / (x * sumX2 - sumX * sumX);
//						printf("id:[%d]fa:[%.1f]\n",index,nb->fa_kinzi[index]);
						seppen = (sumX2 * sumY - sumXY * sumX) / (x * sumX2 - sumX * sumX);

						if(nb->fa_kinzi[index] < 0.0 ){
							nb->kinzi[index] = fabs(nb->fa_kinzi[index]);
						}

						if(nb->kinzi[index] == 0.0 ){
							nb->kinzi[index] = 0.1;
						}

//						printf("id:[%d]kinzi:[%.1f]\n",index,nb->kinzi[index]);
						denpakeisan = denpaarea - nb->nb_distance[index][q]; //現時点での端末の距離を取得
//						printf("time:[%.10f]\n",CURRENT_TIME);
//						printf("id:[%d]kyori:[%.1f]\n",index,denpakeisan);
						nb->waf =  denpakeisan / nb->kinzi[index]; //何秒後に切断するか計算
//						printf("id:[%d]waf:[%.1f]\n",index,nb->waf);
						nb->st_time = CURRENT_TIME + nb->waf; //何秒後に切断するか計算+現在時刻と付け加える
						printf("id:[%d]st_time1hana:[%.1f]kinzi[%.1f]\n",index,nb->st_time,nb->kinzi[index]);
					}
				}
				else if(nb->nb_distance[index][q] < nb->nb_distance[index][q-1] ){ //近づく場合
//				if(nb->nb_distance[index][q] < nb->nb_distance[index][q-1] ){ //近づく場合
					on1 = 1;
					x = 0, y = 0, sumXY = 0, sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, seppen = 0;
					double on[100] = {0};
					for(int t = q; nb->nb_distance[index][t] < nb->nb_distance[index][t-1] && t >= 0; t--){
						on[on1-1] = nb->nb_distance[index][t];
//						printf("tika-st\n");
//						printf("[on:%d]\n",on1);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,on[0]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,on[1]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,on[2]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,on[3]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,on[4]);
 
                        x = on1;
                        y = on[on1-1];
                        sumXY += x * y;
                        sumX  += x;
                        sumY  += y;
                        sumX2 += x * x;
                        sumY2 += y * y;
//						printf("[on1:%d]\n",on1);
//						printf("[x:%.5f]\n",x);
//						printf("[y:%.5f]\n",y);
//						printf("[xy:%.5f]\n",sumXY);
//						printf("[sumx:%.5f]\n",sumX);
//						printf("[sumy:%.5f]\n",sumY);
//						printf("[x2:%.5f]\n",sumX2);
//						printf("[y2:%.5f]\n",sumY2);
						on1++;
					}
					if(on1 > 2){
//						printf("t-kinzi-start\n");
//						printf("[on:%d]\n",on1);
//						printf("[x:%.5f]\n",x);
//						printf("[y:%.5f]\n",y);
//						printf("[xy:%.5f]\n",sumXY);
//						printf("[sumx:%.5f]\n",sumX);
//						printf("[sumy:%.5f]\n",sumY);
//						printf("[x2:%.5f]\n",sumX2);
//						printf("[y2:%.5f]\n",sumY2);

//						printf("a%.5f\n",(x * sumXY));
//						printf("aa%.5f\n",(sumX * sumY));
//						printf("aaa%.5f\n",((x * sumXY) - (sumX * sumY)));
//						printf("b%.5f\n",(x * sumX2));
//						printf("bb%.5f\n",(sumX * sumX));
//						printf("bbb%.5f\n",((x * sumX2) - (sumX * sumX)));
//						printf("cc%.5f\n",(x * sumXY - sumX * sumY) / (x * sumX2 - sumX * sumX));
						nb->kinzi[index] = ((x * sumXY - sumX * sumY) / (x * sumX2 - sumX * sumX));
//						printf("id:[%d]kinzi:[%.1f]\n",index,nb->kinzi[index]);
						seppen = (sumX2 * sumY - sumXY * sumX) / (x * sumX2 - sumX * sumX);

						if(nb->kinzi[index] < 0.0 ){
							nb->go_kinzi[index] = fabs(nb->kinzi[index]);
						}
						
						if(nb->kinzi[index] == 0.0 ){
							nb->go_kinzi[index] = 0.1;
						}

						if(nb->kinzi[index] > 0.0 ){
							nb->tog1_kinzi[index] = ((denpaarea - on[0]) / nb->kinzi[index]); //最小
							nb->tog2_kinzi[index] = denpaarea / nb->kinzi[index]; //最大
							nb->tog3_kinzi[index] = on[0] / nb->kinzi[index]; //折り返し最大
							nb->tog4_kinzi[index] = nb->tog2_kinzi[index] + nb->tog3_kinzi[index];
//							printf("id:[%d]to1-kinzi:[%.1f]\n",index,nb->tog1_kinzi[index]);
//							printf("id:[%d]to2-kinzi:[%.1f]\n",index,nb->tog2_kinzi[index]);
//							printf("id:[%d]to3-kinzi:[%.1f]\n",index,nb->tog3_kinzi[index]);
//							printf("id:[%d]to4-kinzi:[%.1f]\n",index,nb->tog4_kinzi[index]);

							nb->waf = ((nb->tog1_kinzi[index] + nb->tog4_kinzi[index]) / 2); 
//							printf("id:[%d]waf:[%.1f]\n",index,nb->waf);
							nb->st_time = CURRENT_TIME + nb->waf; //何秒後に切断するか計算+現在時刻と付け加える
							printf("id:[%d]st_time2tika:[%.1f]kinzi[%.1f]\n",index,nb->st_time,nb->kinzi[index]);
						}
					}
				}
				else if(nb->nb_distance[index][q] == nb->nb_distance[index][q-1]  && nb->nb_distance[index][q] > 0){ //同じ場合
					sa1 = 1;
					x = 0, y = 0, sumXY = 0, sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, seppen = 0;
					double same[100] = {0};
					for(int s = q; nb->nb_distance[index][s] > 0 && s >= 0; s--){
						same[sa1-1] = nb->nb_distance[index][s];
						nb->st_time = CURRENT_TIME + 100; //何秒後に切断するか計算+現在時刻と付け加える
						printf("id:[%d]st_time3onaji:[%.1f]\n",index,nb->st_time);
//						printf("?????\n");
//						printf("[sa:%d]\n",sa1);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,same[0]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,same[1]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,same[2]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,same[3]);
//						printf("[time:%.5f][kyori:%.1f]\n",CURRENT_TIME,same[4]);
					} 
				}
			}
		}
     printf("id:[%d]\n",index);
 for(int w = 0; w < nb->app_count[index]; ++w){
     printf("kyori:[%.1f]",nb->nb_distance[index][w]);
  } 
printf("\n");
#endif //PROP
//------------------------------//

#ifdef MISU
rtable.rt_dump(index);
nb_dump(index);
#endif //MISU

	} //1591
 Packet::free(p);
}


//#ifdef KIZON
//void
//AODV::nb_insert(nsaddr_t id) {
//#endif //KIZON

//#ifdef PROP
void
AODV::nb_insert(nsaddr_t id, Packet *p) {
//#endif //PROP
AODV_Neighbor *nb = new AODV_Neighbor(id);

 assert(nb);
 nb->nb_expire = CURRENT_TIME +
                (1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);

//tuika------------------------------//
#ifdef PROP
	for(int i = 0; nb->nb_distance[index][i] != 0; i++){ //ネイバーテーブルの初期化・更新処理
	nb->nb_distance[index][i] = 0.0; 
	}

//	double dbm = 10*log10(p->txinfo_.RxPr); //dbm変換
	double lambda = 300000000/914e+6;
	double PI = 3.14159265359;
	double distan = sqrt((0.28183815 * 1 * 1 * lambda * lambda) / (1 * (p->txinfo_.RxPr))) / (4 * PI); 
	distance = round(distan);

	//nb->nb_distance;
	//app_distance[N] = distance;
	//printf("app_distance %.20f\n",app_distance[0+N]);
	//printf("app_time %.20f\n",app_time[0]);
#endif //PROP
//------------------------------//


 LIST_INSERT_HEAD(&nbhead, nb, nb_link);
 seqno += 2;             // set of neighbors changed
 assert ((seqno%2) == 0);
}


AODV_Neighbor*
AODV::nb_lookup(nsaddr_t id) {
AODV_Neighbor *nb = nbhead.lh_first;

 for(; nb; nb = nb->nb_link.le_next) {
   if(nb->nb_addr == id) break;
 }
 return nb;
}


/*
 * Called when we receive *explicit* notification that a Neighbor
 * is no longer reachable.
 */
void
AODV::nb_delete(nsaddr_t id) {
AODV_Neighbor *nb = nbhead.lh_first;

 log_link_del(id);
 seqno += 2;     // Set of neighbors changed
 assert ((seqno%2) == 0);

 for(; nb; nb = nb->nb_link.le_next) {
   if(nb->nb_addr == id) {
     LIST_REMOVE(nb,nb_link);
     delete nb;
     break;
   }
 }

 handle_link_failure(id);

}


/*
 * Purges all timed-out Neighbor Entries - runs every
 * HELLO_INTERVAL * 1.5 seconds.
 */
void
AODV::nb_purge() {
AODV_Neighbor *nb = nbhead.lh_first;
AODV_Neighbor *nbn;
double now = CURRENT_TIME;

 for(; nb; nb = nbn) {
   nbn = nb->nb_link.le_next;
   if(nb->nb_expire <= now) {
     nb_delete(nb->nb_addr);
   }
 }

}
