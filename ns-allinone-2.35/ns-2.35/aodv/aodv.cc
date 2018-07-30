
//#include <ip.h>

#include <aodv/aodv.h>
#include <aodv/aodv_packet.h>
#include <random.h>
#include <cmu-trace.h>
#include "mobilenode.h"

#include <iostream>


//AddStart2016/12/18
//calc_threshold内の変数を呼び出すためにインクルードする
#include "calc_threshold.h"

#include <energy-model.h>

#include <unistd.h>


#define max(a,b)        ( (a) > (b) ? (a) : (b) )
#define CURRENT_TIME    Scheduler::instance().clock()

//#define DEBUG
//#define ERROR
#define RREQN  //nagumo追加　長田さんのプログラム参考にして
#define ne-AODV //nagumo　距離、ID送信
//#define REPLY //中間リプレイ動かないようにする コメントアウト消すと動くようになる
//#ifdef aaaa と #endif で挟んだところがコメントアウトされる　上のコメントアウト消すと挟んでも動く

#ifdef DEBUG
static int extra_route_reply = 0;
static int limit_route_erequest = 0;
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
      //HELLOパケットが見えるようにコメントにする
      //#ifndef AODV_LINK_LAYER_DETECTION
      htimer.handle((Event*) 0);
      ntimer.handle((Event*) 0);
      //#endif // LINK LAYER DETECTION

      rtimer.handle((Event*) 0);
      return TCL_OK;
    }               

    //1018 追加
    if(strcmp(argv[1], "show-node-energy") == 0) {

      double my_energy;

      my_energy = thisnode->energy_model()->energy();
      return TCL_OK;
    }
    if(strcmp(argv[1], "show-node-energy") == 0) {

      double my_energy;

      my_energy = thisnode->energy_model()->energy();
      return TCL_OK;
    }
    if(strcmp(argv[1], "show-node-info") == 0) {

      double x_position, y_position, z_position;
      double x_speed, y_speed, z_speed;
      double my_energy;

      thisnode->getLoc(&x_position, &y_position, &z_position);
      thisnode->getVelo(&x_speed, &y_speed, &z_speed);
      my_energy = thisnode->energy_model()->energy();
      return TCL_OK;
    }
    //1018ここまで

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

    LIST_INIT(&nbhead);
    LIST_INIT(&bihead);

    logtarget = 0;
    ifqueue = 0;

    //追加　nagumo "id"は、indexの別名であり、AODVのIPアドレスを表す
    thisnode = (MobileNode *)(Node::get_node_by_address(id));

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
    fprintf(stderr,"Node %d: Dst - %d, failed local repair\n",index, rt->rt_dst);
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
    if (ch->ptype() != PT_TCP && ch->ptype() != PT_ACK) {
      ch->size() += IP_HDR_LEN;
    }
    // Added by Parag Dadhania && John Novatnack to handle broadcasting
    if ( (u_int32_t)ih->daddr() != IP_BROADCAST) {
      ih->ttl_ = NETWORK_DIAMETER;
    }
  }
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

  my_energy = thisnode->energy_model()->energy(); //バッテリ残量を取得する　nagumo

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

  if (id_lookup(rq->rq_src, rq->rq_bcast_id)) { //すでにRREQパケットを受け取ったことがある場合はTrue（重複受信処理）を行う

#ifdef DEBUG
    fprintf(stderr, "%s: discarding request\n", __FUNCTION__);
#endif // DEBUG


    //追加 nagumo
#ifdef RREQN
    if(rq->rq_dst != index) // 自分がRREQのあて先ノードではない場合の処理
    {
      Packet::free(p); // RREQを破棄
      return;
    }
    else
    { // 自分がRREQのあて先ノードである場合の処理
      //if( (dst_timeout[rq->rq_src][rq->rq_bcast_id] > CURRENT_TIME)&& (rq->rq_min_energy>dst_min_energy[rq->rq_src][rq->rq_bcast_id]) )
      if( (dst_timeout[rq->rq_src][rq->rq_bcast_id] > CURRENT_TIME)&& (rq->rq_sara>dst_min_energy[rq->rq_src][rq->rq_bcast_id]) )
      { // はじめのRREQを受信してからTwait(50[ms])秒経過していない(タイマー待ち時間内)であり、かつ、このRREQ中の最低バッテリ値が今までの最良のものよりも好条件であるならば

        if(dst_hop_count[rq->rq_src][rq->rq_bcast_id] >= rq->rq_hop_count){
      //    printf("[%d]",rq->rq_bcast_id);
        //  printf("[%4.5f](sec)_______________Node[%d]がNode[%d]からRREQを重複受信しました。このRREQ中のさらし端末数[%d]は現在の最大値[%4f]よりも好条件である。\n",CURRENT_TIME,index, ih->saddr(),rq->rq_sara,dst_min_energy[rq->rq_src][rq->rq_bcast_id]);
          //printf("sara%d\n" , rq->rq_sara );
          //dst_min_energy[rq->rq_src][rq->rq_bcast_id] = rq->rq_min_energy; // 現在までの経路最小バッテリ値の最大値を更新する
          dst_min_energy[rq->rq_src][rq->rq_bcast_id] = rq->rq_sara; // 現在までの経路最小バッテリ値の最大値を更新する
          //printf("[%4.5f](sec)_____________Node[%d]です。送信予定のRREPをキャンセルします。(キャンセルパケットのアドレス=[%d] ([rq_src]= [%d], [rq_bcast_id]=[%d]))\n",CURRENT_TIME, index,dst_pkt_ptr[rq->rq_src][rq->rq_bcast_id], rq->rq_src,rq->rq_bcast_id);
          ListScheduler::instance().cancel( dst_pkt_ptr[rq->rq_src][rq->rq_bcast_id] ); // すでにスケジューリング済みのRREPパケットをキャンセルする!!!!
          /////////////////////// 探索元へのリバース経路を、新しい経路へとアップデートする //////////////////////////
          aodv_rt_entry *rt0; // rt0 is the reverse route
          rt0 = rtable.rt_lookup(rq->rq_src);
          if(rt0 == 0) { /* if not in the route table */
            // create an entry for the reverse route.
            rt0=
              rtable.rt_add(rq->rq_src);
          }rt0->rt_expire = max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE));
          // If we have a fresher seq no. or lesser #hops for the
          // same seq no., update the rt entry. Else don't bother.
          rt_update(rt0, rq->rq_src_seqno,rq->rq_hop_count,
              ih->saddr(),max(rt0->rt_expire, (CURRENT_TIME + REV_ROUTE_LIFE)) );
          if (rt0->rt_req_timeout> 0.0) {
            //Reset the soft state and
            //Set expiry time to CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT
            //This is because route is used in the forward direction,
            //but only sources get benefited by this change
            rt0->rt_req_cnt =
              0;rt0->rt_req_timeout =
              0.0; rt0->rt_req_last_ttl=
              rq->rq_hop_count;rt0->rt_expire = CURRENT_TIME + ACTIVE_ROUTE_TIMEOUT;
          }
          //////////////////////////////////////////////////////////////////////////////////////////////////////////////
          sendReply(rq->rq_src, // IP Destination
              1, // Hop Count
              index, // Dest IP Address
              seqno, // Dest Sequence Num
              MY_ROUTE_TIMEOUT, // Lifetime
              rq->rq_timestamp, // timestamp
              rq->rq_src, // index
              rq->rq_bcast_id, // bcast_id
              dst_timeout[rq->rq_src][rq->rq_bcast_id] - CURRENT_TIME);
          //RREPの送信をリスケジューリングする。送信が開始される時刻dst_timeoutから、現在時間をさし引いた「dst_timeout[ ][ ] - CURRENT_TIME 秒後」に送信されるようスケジューリングする。
          //printf("[%4.5f](sec)_____________Node[%d]です。新しいRREPを[%4f]秒後に送信します。\n",CURRENT_TIME,index,dst_timeout[rq->rq_src][rq->rq_bcast_id] - CURRENT_TIME);
          Packet::free(p);
          return;
        }
      }
      else
      { // はじめのRREQを受信してからすでにTwait秒以上経過している。もしくは、このRREQ中の最低バッテリ値が、現在の最良値よりも小さい(悪い)場合
        Packet::free(p);
        return;
      }
    }



#endif






    Packet::free(p);
    return;
  }

  /*
   * Cache the broadcast ID -初回受信時の処理を行う-
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

  if(rq->rq_dst == index) {   //RREQ初回到達時処理　初回必ずこの処理を行うため変数はここに書く　nagumo
    dst_hop_count[rq->rq_src][rq->rq_bcast_id] = rq->rq_hop_count;
#ifdef DEBUG
    fprintf(stderr, "%d - %s: destination sending reply\n",
        index, __FUNCTION__);
#endif // DEBUG

    //	printf("[%4.5f](sec)    node[%d]が受信しました。node[%d]より、初めての-RREQを受信しました。現在の経路中の最低バッテリ残量値は[%4.5f]です。\n",CURRENT_TIME, index, ih->saddr(),rq->rq_min_energy); //追加nagumo
    //	printf("[%4.5f](sec)    node[%d]が受信しました。node[%d]より、初めての-RREQを受信しました。現在の経路中の最低さらし端末数は[%d]です。\n",CURRENT_TIME, index, ih->saddr(),rq->rq_sara); //追加nagumo

    //	printf("sara%d\n" , rq->rq_sara );
    //	dst_min_energy[rq->rq_src][rq->rq_bcast_id] = rq->rq_min_energy; //経路の最低バッテリ値rq_min_energyを保存　nagumo　追加
    dst_min_energy[rq->rq_src][rq->rq_bcast_id] = rq->rq_sara; //さらし端末数宇を保存　nagumo　追加
    //dst_min_energy[][]は、ノード[rq->rq_src]から送信された[rq->rq_bcast_id]番目のRREQについて、現在到着した中で、最低バッテリ値の最も大きい経路の最低バッテリ値を格納する変数　nagumo
    //cout << "korehanani???:" << dst_min_energy[rq->rq_src][rq->rq_bcast_id] << endl;
    dst_timeout[rq->rq_src][rq->rq_bcast_id] = Twait + CURRENT_TIME;  //タイムアウトする時刻の算出　nagumo　追加

    // Just to be safe, I use the max. Somebody may have
    // incremented the dst seqno.
    seqno = max(seqno, rq->rq_dst_seqno)+1;
    if (seqno%2) seqno++;

    sendReply(rq->rq_src,           // IP Destination
        1,                    // Hop Count
        index,                // Dest IP Address
        seqno,                // Dest Sequence Num
        MY_ROUTE_TIMEOUT,     // Lifetime
        rq->rq_timestamp,     // timestamp
        rq->rq_src,           //index　追加　nagumo
        rq->rq_bcast_id,      //bcast_id 追加　nagumo
        Twait);               // Twait(50[ms])後に送信される

    Packet::free(p);
  }

  // I am not the destination, but I may have a fresh enough route.
#ifdef REPLY  //中間リプレイ nagumo

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

    //#ifdef RREQ_GRAT_RREP  

    sendReply(rq->rq_dst,
        rq->rq_hop_count,
        rq->rq_src,
        rq->rq_src_seqno,
        (u_int32_t) (rt->rt_expire - CURRENT_TIME),
        //             rt->rt_expire - CURRENT_TIME,
        rq->rq_timestamp);
    //#endif

    // TODO: send grat RREP to dst if G flag set in RREQ using rq->rq_src_seqno, rq->rq_hop_counT

    // DONE: Included gratuitous replies to be sent as per IETF aodv draft specification. As of now, G flag has not been dynamically used and is always set or reset in aodv-packet.h --- Anant Utgikar, 09/16/02.

    Packet::free(p);
  }

#endif  //REPLY中間リプレイここまで　nagumo


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

  struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);  //ポインタrqを通してRREQヘッダにアクセスできるようにする　nagumo
  my_energy=thisnode->energy_model()->energy();    //自分のバッテリ残量値を取得する　nagumo

  AODV_Neighbor *nb = nbhead.lh_first; //nagumo ネイバ追加？

  //rq->rq_sarag = rt->rt_sara;
  //	printf("saraggggggggggg%d\n" , rq->rq_sarag );


  //	printf("nfdonononononononononononR\t\t:\t%d\n",rq->rq_nodeid);//ネイバーリストに格納されている通信先 nagumo

  if(ih->ttl_ == 0) {

#ifdef DEBUG
    fprintf(stderr, "%s: calling drop()\n", __PRETTY_FUNCTION__);
#endif // DEBUG

    drop(p, DROP_RTR_TTL);
    return;
  }

  if (ch->ptype() != PT_AODV && ch->direction() == hdr_cmn::UP &&
      ((u_int32_t)ih->daddr() == IP_BROADCAST)
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
    if (ch->ptype() == PT_AODV) {
      /*
       *  Jitter the sending of AODV broadcast packets by 10ms
       */ 
      if(rq->rq_min_energy > my_energy){      //もしも自分のバッテリ残量が、現在のRREQ中の最小バッテリ残量値よりも少なければ、nagumo
        rq->rq_min_energy = my_energy;  //RREQ内のrq_min_energyフィールドを自身のバッテリ残量値に更新する　nagumo
        //}printf("[%4.5f](sec)   node[%d]がRREQを中継します。現在の経路バッテリ残量値は[%4.5f]です。\n",CURRENT_TIME, index, rq->rq_min_energy);  //追加nagumo
    }
    //printf("[%4.5f](sec)   node[%d]がRREQを中継します。現在の経路さらし端末は[%d]です。\n",CURRENT_TIME, index, rq->rq_sarag);  //追加nagumo



    //nagumo
    //	printf("Neighbor list of Node ID :%d\n",index);
    for(; nb; nb = nb->nb_link.le_next) {
      //rq->rq_id_distance[nb->nb_addr][nb->nb_mdistance];
      //   rq->rq_mdistance	=nb->nb_mdistance;
      //	rq->rq_nodeid		=nb->nb_addr;
      //		printf("Node ADDR\t\t:\t%d\n",nb->nb_addr);//ネイバーリストに格納されている通信先
      //		printf("nononR\t\t:\t%d\n",rq->rq_nodeid);//ネイバーリストに格納されている通信先
      //		printf("nb_expire\t\t:\t%f\n",nb->nb_expire);//ネイバーリスト保持時間
      //		printf("nb_distance\t:\t%f\n",nb->nb_distance);//その通信先までの距離
      //		printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH 
      //		printf("nb_mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
      //		printf("mmmmmmmmmmmmmmmm(w) (%0.2fm)\n", rq->rq_mdistance ); //RSCH nagumo
      //	printf("mmmmmmmmmmmmmmmm(w) (%0.2fm)\n", rq->rq_exposure ); //RSCH nagumo
      //		printf("---------------------------------\n");
      //if(nb->nb_addr == id) break;
      if(rq->rq_nodeid == nb->nb_addr){
        double dista = nb->nb_mdistance;

        //		printf("test (w) (%0.2fm)\n", dista ); //RSCH nagumo 
        //		printf("test mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo 
        rq->rq_nodeid = index;
        if(dista < nb->nb_mdistance ){
          //rq->rq_exposure++;
          //	printf("さらし端末数\n",rq->rq_exposure);
        }
      }
    }

    //printf("------------------------------------\n");


    /*
       for(; nb; nb = nb->nb_link.le_next) {
       if(rq->rq_nodeid == nb->nb_addr){

       printf("test mdistance(w) (%0.2fm)\n", rq->rq_mdistance ); //RSCH nagumo 
       }
       }
       */
    //for(int k = 0;rq->rq_testlast;k++){
    //printf(rq->rq_exposure_list[]);
    //}



    //int i = 0;

    //	for(; nb; nb = nb->nb_link.le_next) {




    /*
       printf("距離%f\n", rq->rq_exposure_list[0] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[1] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[2] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[3] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[4] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[5] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[6] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[7] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[8] );   //01/10 nagumo
       printf("距離%f\n", rq->rq_exposure_list[9] );   //01/10 nagumo
       printf("ノードID%d\n" , rq->rq_testlast_list[0]);
       printf("ノードID%d\n" , rq->rq_testlast_list[1]);
       printf("ノードID%d\n" , rq->rq_testlast_list[2]);
       printf("ノードID%d\n" , rq->rq_testlast_list[3]);
       printf("ノードID%d\n" , rq->rq_testlast_list[4]);
       printf("ノードID%d\n" , rq->rq_testlast_list[5]);
       printf("ノードID%d\n" , rq->rq_testlast_list[6]);
       printf("ノードID%d\n" , rq->rq_testlast_list[7]);
       printf("ノードID%d\n" , rq->rq_testlast_list[8]);
       printf("ノードID%d\n" , rq->rq_testlast_list[9]);
       */
    //	i++;
    //}




    //nagumo 送られてきたネイバーから自身（中継ノード）のノードIDを探している
    int M =0;
    for(int m = 0; m < 100; m++){  //ノード数と配列の長さで回す数を変更
      if(rq->rq_testlast_list[m] == index){
        M = m;

        //	printf("ノードIDaaaaaaaaaaaaaa%d\n" , M);
      }
      //	printf("ノードIDaa%d\n" , rq->rq_testlast_list[m]);

    }



    //さらし端末をカウントしている
    double dista = rq->rq_exposure_list[M];

    for(int s = 0; s <100; s++ ){
      if(rq->rq_exposure_list[s] > dista){ //自身の距離より離れた（距離の大きい）ノードをカウントしている
        rq->rq_sara++;
        //	printf("sara%d\n" , rq->rq_sara );
      }
    }
    rq->rq_sarag = rq->rq_sara;

    //	printf("saraggggg%d\n" , rq->rq_sarag );

    //中継端末のネイバー情報を送信
    int mm = 0;
    //tuika nagumo
    //printf("Neighbor list of Node ID :%d\n",index);
    for(; nb; nb = nb->nb_link.le_next) {

      rq->rq_exposure_list[mm] = nb->nb_mdistance;   //01/10 nagumo
      rq->rq_testlast_list[mm] = nb->nb_addr;   //01/10 nagumo
      /*			printf("距離%0.2f\n" , rq->rq_exposure_list[mm]);
              printf("lllllllllllllllllllllllllll%d\n" , nb->nb_addr);
              printf("ノードID%d\n" , rq->rq_testlast_list[mm]);
              */
      mm++;
    }
    //	printf("------------------------------------\n");









    //rt->rt_sara = rq->rq_sara;


    Scheduler::instance().schedule(target_, p,
        0.01 * Random::uniform());
  } else {
    Scheduler::instance().schedule(target_, p, 0.);  // No jitter
  }
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
  //printf("test",rq->rq_exposures); //nagumo
  Packet *p = Packet::alloc();
  struct hdr_cmn *ch = HDR_CMN(p);
  struct hdr_ip *ih = HDR_IP(p);
  struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);
  aodv_rt_entry *rt = rtable.rt_lookup(dst);

  AODV_Neighbor *nb = nbhead.lh_first; //nagumo ネイバ追加？

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
  // RREQパケットを記入
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
  //rq->rq_exposures = 10; //nagumo

  //nagumo 
  my_energy=thisnode->energy_model()->energy(); //ノードの持つバッテリー残量を取得する
  rq->rq_min_energy = my_energy;                //経路中の最小バッテリーとして自身のバッテリ残量を格納
  //printf("[%4.5f](sec)_______________Node[%d]が、あて先[%d]に対してRREQを送信します。なお、Node[%d]の現在のバッテリ残量は[%4.5f]です。 \n",CURRENT_TIME, index, dst, index,rq->rq_min_energy);

  //nagumo
  rq->rq_sara = 0;
  rq->rq_sarag = 0;

  //		printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH  nagumo

  int kk = 0;
  //tuika nagumo
  //	printf("Neighbor list of Node ID :%d\n",index);
  for(; nb; nb = nb->nb_link.le_next) {

    //rq->rq_exposure = nb->nb_mdistance;   //01/10 nagumo

    //		printf("Node ADDR\t\t:\t%d\n",nb->nb_addr);//ネイバーリストに格納されている通信先
    //	printf("nfdonononononononononononR\t\t:\t%d\n",rq->rq_nodeid);//ネイバーリストに格納されている通信先
    //		printf("nb_expire\t\t:\t%f\n",nb->nb_expire);//ネイバーリスト保持時間
    //		printf("nb_distance\t:\t%f\n",nb->nb_distance);//その通信先までの距離
    //		printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH 
    //		printf("nb_mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
    //		printf("mmmmmmmmmmmmmmmm(w) (%0.2fm)\n", rq->rq_mdistance ); //RSCH nagumo
    //		printf("---------------------------------\n");
    //if(nb->nb_addr == id) break;
    rq->rq_exposure_list[kk] = nb->nb_mdistance;   //01/10 nagumo
    rq->rq_testlast_list[kk] = nb->nb_addr;   //01/10 nagumo
    //	printf("距離%0.2f\n" , rq->rq_exposure_list[kk]);
    //	printf("lllllllllllllllllllllllllll%d\n" , nb->nb_addr);
    //	printf("ノードID%d\n" , rq->rq_testlast_list[kk]);
    kk++;
  }
  //	printf("------------------------------------\n");

  /*
     printf("距離%f\n", rq->rq_exposure_list[0] );   //01/10 nagumo
     printf("距離%f\n", rq->rq_exposure_list[1] );   //01/10 nagumo
     printf("距離%f\n", rq->rq_exposure_list[2] );   //01/10 nagumo
     printf("距離%f\n", rq->rq_exposure_list[3] );   //01/10 nagumo
     printf("pppppppppppp%d\n" , rq->rq_testlast_list[0]);
     printf("pppppppppppp%d\n" , rq->rq_testlast_list[1]);
     printf("pppppppppppp%d\n" , rq->rq_testlast_list[2]);
     printf("pppppppppppp%d\n" , rq->rq_testlast_list[3]);
     i++;
     }

*/
//int i = 0;

//	for(; nb; nb = nb->nb_link.le_next) {

//	rq->rq_exposure_list[i] = nb->nb_mdistance;   //01/10 nagumo
//	i++;
//	printf("lllllllllllllllllllllllllll%d\n" , rq->rq_exposure_list[i]);
//}
//rq->rq_testlast = i;

#ifdef ne-AODV
/*
   int i = 0;

   for(; nb; nb = nb->nb_link.le_next) {

   rq->rq_exposure_list[i] = nb->nb_mdistance;   //01/10 nagumo
   i++;
   }
   */

//	double	distance;
//	};
//	neighbor_list nlist[100];

//rq_mdistance	=nb->nb_mdistance;
//rq_nodeid		=nb->nb_addr;
//	printf("Neighbor list of Node ID 自身ノードID:%d\n",index);
//	for(; nb; nb = nb->nb_link.le_next) {
//    rq->rq_mdistance	=nb->nb_mdistance;
//	rq->rq_nodeid		=nb->nb_addr;
//		printf("nb_distanceaaaaaaaaaaaaaaaaaa\t:\t%f\n",nb->nb_mdistance);//その通信先までの距離
//}

#endif










//RREQパケット送信時に、自ノードの電波強度の値を格納する
//ADD_START 2016/10/18
//パケット電波強度参照　：　p->txinfo_.RxPr
//rq->rq_radio_power = ？
//ADD END

Scheduler::instance().schedule(target_, p, 0.);
//

}

void
AODV::sendReply(nsaddr_t ipdst, u_int32_t hop_count, nsaddr_t rpdst,
    u_int32_t rpseq, u_int32_t lifetime, double timestamp, int rq_src, int rq_bcast_id, double dst_wait) {//変数追加 nagumo
  Packet *p = Packet::alloc();
  struct hdr_cmn *ch = HDR_CMN(p);
  struct hdr_ip *ih = HDR_IP(p);
  struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);
  aodv_rt_entry *rt = rtable.rt_lookup(ipdst);

  dst_pkt_ptr[rq_src][rq_bcast_id] = p;  //スケジューリングしたRREPパケットの先頭アドレスを保存 nagumo

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

  //Scheduler::instance().schedule(target_, p, 0.); 
  Scheduler::instance().schedule(target_, p, dst_wait);
  //nagumoＲＲＥＱ受付時間設定dst_wait秒に設定 場所 aodv.h 2017/07/13

}

void
AODV::sendError(Packet *p, bool jitter) {
  struct hdr_cmn *ch = HDR_CMN(p);
  struct hdr_ip *ih = HDR_IP(p);
  struct hdr_aodv_error *re = HDR_AODV_ERROR(p);

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
//ハローパケット送信時に呼び出される処理
AODV::sendHello() {
  //パケットオブジェクトを割り当て

  Packet *p = Packet::alloc();
  AODV_Neighbor *nb = nbhead.lh_first;  //20180704tuika　ネイバーリストオブジェクトの割当て？
  struct hdr_cmn *ch = HDR_CMN(p);
  struct hdr_ip *ih = HDR_IP(p);
  struct hdr_aodv_reply *rh = HDR_AODV_REPLY(p);
  struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);  //ポインタrqを通してRREQヘッダにアクセスできるようにする　nagumo
  //aodv_rt_entry *rt;

#ifdef DEBUG
  fprintf(stderr, "sending Hello from %d at %.2f\n", index, Scheduler::instance().clock());
#endif // DEBUG

  rh->rp_type = AODVTYPE_HELLO;	//パケットタイプ指定
  //rh->rp_flags = 0x00;
  rh->rp_hop_count = 1;			//ルーティングパケットのホップカウントを１にする（helloパケットは隣接ノードにのみ渡されるため？）
  rh->rp_dst = index;			//送信先をindex（ノードID）に指定。隣接ノードへのブロードキャストなのに？
  rh->rp_dst_seqno = seqno;
  rh->rp_lifetime = (1 + ALLOWED_HELLO_LOSS) * HELLO_INTERVAL;

  // ch->uid() = 0;
  ch->ptype() = PT_AODV;
  ch->size() = IP_HDR_LEN + rh->size(); //パケットサイズの変更（IPヘッダ＋Helloパケットのサイズ）
  ch->iface() = -2;
  ch->error() = 0;
  ch->addr_type() = NS_AF_NONE;
  ch->prev_hop_ = index;			// AODV hack	←意味がよくわからない

  ih->saddr() = index;			// IPヘッダの送信index ノード番号を付与
  ih->daddr() = IP_BROADCAST;
  ih->sport() = RT_PORT;
  ih->dport() = RT_PORT;
  ih->ttl_ = 1;

/*
  for(; nb; nb = nb->nb_link.le_next) {

    if(rq->rq_nodeid == nb->nb_addr){
      double dista = nb->nb_mdistance;

      //    printf("test (w) (%0.2fm)\n", dista ); //RSCH nagumo
      //    printf("test mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
      rq->rq_nodeid = index;
      if(dista < nb->nb_mdistance ){

      }
    }
  }



  int M =0;
  for(int m = 0; m < 100; m++){  //ノード数と配列の長さで回す数を変更
    if(rq->rq_testlast_list[m] == index){
      M = m;
    }
  };
  double dista = rq->rq_exposure_list[M];

  for(int s = 0; s <100; s++ ){
    if(rq->rq_exposure_list[s] > dista){ //自身の距離より離れた（距離の大きい）ノードをカウントしている
      rq->rq_sara++;
      printf("sara%d\n" , rq->rq_sara );
    }
  }
  rq->rq_sarag = rq->rq_sara;

  printf("saraggggg%d\n" , rq->rq_sarag );




  int kk =0;
  for(; nb; nb = nb->nb_link.le_next) {

    //rq->rq_exposure = nb->nb_mdistance;   //01/10 nagumo

    //    printf("Node ADDR\t\t:\t%d\n",nb->nb_addr);//ネイバーリストに格納されている通信先
    //  printf("nfdonononononononononononR\t\t:\t%d\n",rq->rq_nodeid);//ネイバーリストに格納されている通信先
    //    printf("nb_expire\t\t:\t%f\n",nb->nb_expire);//ネイバーリスト保持時間
    //    printf("nb_distance\t:\t%f\n",nb->nb_distance);//その通信先までの距離
    //    printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH
    //    printf("nb_mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
    //    printf("mmmmmmmmmmmmmmmm(w) (%0.2fm)\n", rq->rq_mdistance ); //RSCH nagumo
    //    printf("---------------------------------\n");
    //if(nb->nb_addr == id) break;
    rq->rq_exposure_list[kk] = nb->nb_mdistance;   //01/10 nagumo
    rq->rq_testlast_list[kk] = nb->nb_addr;   //01/10 nagumo
    cout << "時間:[" << CURRENT_TIME << "]" << endl;
    cout << "index:" << index << endl;

    printf("距離%0.2f\n" , rq->rq_exposure_list[kk]);
    printf("lllllllllllllllllllllllllll%d\n" , nb->nb_addr);
    printf("ノードID%d\n" , rq->rq_testlast_list[kk]);
    kk++;
  }
  printf("------------------------------------\n");

  int mm = 0;
  //tuika nagumo
  //printf("Neighbor list of Node ID :%d\n",index);
  for(; nb; nb = nb->nb_link.le_next) {

    rq->rq_exposure_list[mm] = nb->nb_mdistance;   //01/10 nagumo
    rq->rq_testlast_list[mm] = nb->nb_addr;   //01/10 nagumo
          printf("距離%0.2f\n" , rq->rq_exposure_list[mm]);
            printf("lllllllllllllllllllllllllll%d\n" , nb->nb_addr);
            printf("ノードID%d\n" , rq->rq_testlast_list[mm]);
            
    mm++;
  }
*/



  /*		printf("sara%d\n" , rq->rq_sara);
        printf("**********************************************\n");
        printf("saraggggg%d\n" , rq->rq_sarag );
        */





    /*if(rq->rq_sara >= 10){
      cout << "nbdetayoooo:" << rq->rq_sara << endl;
      nb_delete();
      }else{

*/
    Scheduler::instance().schedule(target_, p, 0.0);  //ＲＲＥＱ受付時間の設定 場所間違えているぽいね
    //Scheduler::instance().schedule(target_, p, 0.5);  //追加 ＲＲＥＱ受付時間を0.05秒に設定 2017/06/07　nagumo
  
  }


  void
    //AddStart 2016/12/06
    //ハローパケットを受け取った時に呼ばれるメソッド
    //引数はパケットのオブジェクトなので、電波強度はそのまま参照可能
    AODV::recvHello(Packet *p) {
      //struct hdr_ip *ih = HDR_IP(p);
      struct hdr_aodv_reply *rp = HDR_AODV_REPLY(p);//aodv_packet.h内に存在
      struct hdr_aodv_request *rq = HDR_AODV_REQUEST(p);  //ポインタrqを通してRREQヘッダにアクセスできるようにする　nagumo
      AODV_Neighbor *nb;	//aodv_rtable.h内に存在
      //nb_lookupメソッドは、ネイバーリストすべてを最初から最後まで順番に参照し、与えた引数(送信元ノード番号)と
      //すでに保持しているノード番号が一致するか（新しくパケットが来たノードか）を判断している
      nb = nb_lookup(rp->rp_dst);
      if(nb == 0) {
        //ネイバーリストに無いノードからHELLOパケットを受け取った場合は、そのノード情報を、ネイバーリストに追加する
        nb_insert(rp->rp_dst, p);

        //AddStart 2017/01/05
        //ネイバーリストの内容を、mobilenodeオブジェクトが持つ構造体にコピーする
        nb_copy(p);
      }
      else {
        //既にネイバーリストにあるノードからHELLOパケットを受け取った場合、そのノード情報の保持時間を伸ばす
        nb->nb_expire = CURRENT_TIME +
          (1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);            
        //AddStart 2017/01/05
        nb->nb_distance = p->txinfo_.RxPr;
        nb_copy(p);
        
        //printf("aaaaaa\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH nagumo
     
      //nakabayashi start 2018/07/25


      rq->rq_sara = 0;
      rq->rq_sarag = 0;
     
  int kk =0;
  for(; nb; nb = nb->nb_link.le_next) {

       // rq->rq_exposure = nb->nb_mdistance;   //01/10 nagumo

   // cout << "時間:[" << CURRENT_TIME << "]" << endl;
   // cout << "index:" << index << endl;
       // printf("Node ADDR\t\t:\t%d\n",nb->nb_addr);//ネイバーリストに格納されている隣接ノード
       // printf("nfdonononononononononononR\t\t:\t%d\n",rq->rq_nodeid);//ネイバーリストに格納されている通信先
       // printf("nb_expire\t\t:\t%f\n",nb->nb_expire);//ネイバーリスト保持時間
       // printf("nb_distance\t:\t%f\n",nb->nb_distance);//その通信先までの距離
       // printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH
       // printf("nb_mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
       // printf("mmmmmmmmmmmmmmmm(w) (%0.2fm)\n", rq->rq_mdistance ); //RSCH nagumo
        //cout << "dstenation????[" << rq->rq_dst << "]" << endl;
        //cout << "srcccccccc????[" << rq->rq_src << "]" << endl;
        //cout << "bcastId????[" << rq->rq_bcast_id << "]" << endl;
   //     printf("---------------------------------\n");
    //if(nb->nb_addr == id) break;
    rq->rq_exposure_list[kk] = nb->nb_mdistance;   //01/10 nagumo
    rq->rq_testlast_list[kk] = nb->nb_addr;   //01/10 nagumo
    /*
    cout << "時間:[" << CURRENT_TIME << "]" << endl;
    cout << "index:" << index << endl;

    printf("距離%0.2f\n" , rq->rq_exposure_list[kk]);
    printf("lllllllllllllllllllllllllll%d\n" , nb->nb_addr);
    printf("ノードID%d\n" , rq->rq_testlast_list[kk]);
   */
    kk++;
  }
 // printf("===========================================\n");
     
  for(; nb; nb = nb->nb_link.le_next) {

    if(rq->rq_nodeid == nb->nb_addr){
      double dista = nb->nb_mdistance;

      //    printf("test (w) (%0.2fm)\n", dista ); //RSCH nagumo
      //    printf("test mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
      rq->rq_nodeid = index;
      if(dista < nb->nb_mdistance ){

      }
    }
  }
  int M =0;
  for(int m = 0; m < 100; m++){  //ノード数と配列の長さで回す数を変更
    if(rq->rq_testlast_list[m] == index){
      M = m;
    }
  };
  double dista = rq->rq_exposure_list[M];

  for(int s = 0; s <100; s++ ){
    if(rq->rq_exposure_list[s] > dista){ //自身の距離より離れた（距離の大きい）ノードをカウントしている
      rq->rq_sara++;
     // printf("sara%d\n" , rq->rq_sara );
    }

  }

  double hikaku = rq->rq_sara - rq->rq_sarag;



  if(hikaku > 5){ 
  //if(rq->rq_sarag < rq->rq_sara){ 

    //cout << index << "番ノードがなくなりました：時間" << endl;
    nb_delete(index);
    sleep(5);
  }else{

  rq->rq_sarag = rq->rq_sara;
  //cout << index << "番ノードのさらし端末数[" << rq->rq_sarag << "]" << endl;
  }
 // printf("===========================================\n");   

     
     
     
     
     
     
      }

      Packet::free(p);
    }

  void
    //ネイバーリストへの追加を行うメソッド
    AODV::nb_insert(nsaddr_t id,Packet *p) {
      //double dist = rev_threshold( nb->nb_distance ); //RSCH
      //AODV_Neighborクラスを生成、コンストラクタを呼び出す
      AODV_Neighbor *nb = new AODV_Neighbor(id);
      assert(nb);
      nb->nb_expire = CURRENT_TIME +
        (1.5 * ALLOWED_HELLO_LOSS * HELLO_INTERVAL);
      //AddStart 2016/12/06
      //ネイバーリストに、受け取ったHELLOパケットの電波強度から算出した距離(電波強度の差分)を格納する
      //ネイバーリストに、距離を格納するため、calc_threshold.c内のrev_thresholdを呼び出す
      //nb->nb_distance = rev_threshold(p->txinfo_.RxPr); 
      nb->nb_distance = p->txinfo_.RxPr;
      nb->nb_mdistance = rev_threshold(nb->nb_distance);  //nagumo 距離メートル換算
      //printf("nb_distance(w) (%0.2fm)\n", rev_threshold( nb->nb_distance ) ); //RSCH nagumo
      //printf("nb_distance(w) %f\n", rev_threshold( nb->nb_distance ) ); //RSCH nagumo
      //printf("nb_distance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
      LIST_INSERT_HEAD(&nbhead, nb, nb_link);
      seqno += 2;             // set of neighbors changed
      assert ((seqno%2) == 0);

      //AddStart 2016/12/12
      //ネイバーリストの内容を出力させる
      //	nb_display(p);
    }


  AODV_Neighbor*
    AODV::nb_lookup(nsaddr_t id) {
      AODV_Neighbor *nb = nbhead.lh_first;

      for(; nb; nb = nb->nb_link.le_next) {
        if(nb->nb_addr == id) break;
      }
      return nb;
    }

  //AddStart 2016/12/06
  //ネイバーリストの内容を出力する
  //注意点としては、あるノードのネイバーリストに、ノード情報が新しく追加された場合、そのノードが持つネイバーリストの情報を「すべて」出力している
  void
    AODV::nb_display(Packet *p){
      AODV_Neighbor *nb = nbhead.lh_first;
      //printf("[%4.5f](sec) \n",CURRENT_TIME);
      //printf("Neighbor list of Node ID :%d\n",index);
      for(; nb; nb = nb->nb_link.le_next) {
        /*	printf("Node ADDR\t\t:\t%d\n",nb->nb_addr);//ネイバーリストに格納されている通信先
            printf("nb_expire\t\t:\t%f\n",nb->nb_expire);//ネイバーリスト保持時間
            printf("nb_distance\t:\t%f\n",nb->nb_distance);//その通信先までの距離
            printf("nb_distance(w)\t:\t%e (%0.2fm)\n", nb->nb_distance ,rev_threshold( nb->nb_distance ) ); //RSCH 
            printf("nb_mdistance(w) (%0.2fm)\n", nb->nb_mdistance ); //RSCH nagumo
            */
        //		printf("---------------------------------\n");
        //if(nb->nb_addr == id) break;
      }
      //	printf("------------------------------------\n");
    }

  //ネイバーリストの内容を、mac802_11.ccから参照できるように、
  //MobileNodeクラスに追加した構造体に、ネイバーリストの内容をコピーする
  void
    AODV::nb_copy(Packet *p){
      AODV_Neighbor *nb = nbhead.lh_first;
      int i=0;
      for(; nb; nb = nb->nb_link.le_next) {
        God::instance()->MobileObject( addr() )->nlist[i].nodeid = nb->nb_addr;
        //距離のコピー時、しきい値に対してある程度の余裕を持たせて置かないと、recvRTSで弾かれる
        God::instance()->MobileObject( addr() )->nlist[i].distance = nb->nb_distance*0.995;
        /*	
        //追加　10ｍ余裕をもたせる、10ｍおきに10ｍの余裕をもたせる　アバウトなので距離によって持たせる余裕は変わる
        double dist = rev_threshold( nb->nb_distance ); //RSCH
        //double dist = p->txinfo_.RxPr; //RSCH
        //God::instance()->MobileObject( addr() )->nlist[i].distance = 3.60984e-09;//140m
        God::instance()->MobileObject( addr() )->nlist[i].distance = 3.71409e-09*0.9;//140m

        //最低５ｍの余裕をもたせる 追加 17/02/08
        if ( sizeof(nb->nb_addr) > 3 ){  //ネイバーリストの隣接ノード数に応じている
        //printf("addr:%d ",addr());                   //自身のノード番号
        if ( dist < 130){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 4.29566e-09;//135m
        }

        if ( dist < 120){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 5.8442e-09;//125m
        }

        if ( dist < 110){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 8.15781e-09;//115m
        }

        if ( dist < 100){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 1.17384e-08;//105m
        }

        if ( dist < 90){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 1.75174e-08;//95m
        }

        if ( dist < 80){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 2.66129e-08;//85m
        }

        if ( dist < 70){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 3.41828e-08;//75m
        }

        if ( dist < 60){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 4.55096e-08;//65m
        }
        }




        //最低10mの余裕を持たせておく
        if ( dist < 130){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 3.60984e-09;//140m
        }
        if ( dist < 120){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 4.99564e-09;//130m
        }
        if ( dist < 110){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 6.88081e-09;//120m
        }
        if ( dist < 100){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 9.74527e-09;//110m
        }
        if ( dist < 90){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 1.42681e-08;//100m
        }
        if ( dist < 80){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 2.17468e-08;// 90m
        }
        if ( dist < 70){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 3.00435e-08;// 80m
        }
        if ( dist < 60){
        God::instance()->MobileObject( addr() )->nlist[i].distance = 3.92405e-08;// 70m
        }

        //printf("addr:%d ",addr());                   //自身のノード番号
        //printf("nb_addr:%d ",nb->nb_addr);           //ネイバーリストに格納されたノード番号
        //printf("nb_distance:%e\n",nb->nb_distance);  //電波強度
        */		
          i++;
      }
      //ネイバーリストの内容をコピーし終えたら、構造体の最後に終端用の数値を格納する
      God::instance()->MobileObject( addr() )->nlist[i].nodeid = -1;
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
