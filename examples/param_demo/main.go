package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

// tiny helpers
func i64(v int64) *int64     { return &v }
func f64(v float64) *float64 { return &v }

func main() {
	// Init global context (uses default options)
	if err := rclgo.Init(nil); err != nil {
		log.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	n, err := rclgo.NewNode("param_demo", "")
	if err != nil {
		log.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr, err := params.NewManager(n)
	if err != nil {
		log.Fatalf("params.NewManager: %v", err)
	}

	// Declare a few parameters with descriptors
	_, _ = mgr.Declare("camera.fps",
		params.Value{Kind: params.KindInt64, Int64: 15},
		params.Descriptor{MinInt: i64(1), MaxInt: i64(120), Description: "Output frame rate"},
	)
	_, _ = mgr.Declare("camera.frame_id",
		params.Value{Kind: params.KindString, Str: "camera"},
		params.Descriptor{Description: "TF frame for published images"},
	)
	_, _ = mgr.Declare("camera.exposure",
		params.Value{Kind: params.KindDouble, Double: 0.02},
		params.Descriptor{MinDouble: f64(0.001), MaxDouble: f64(0.1)},
	)
	_, _ = mgr.Declare("build.git_sha",
		params.Value{Kind: params.KindString, Str: "dev"},
		params.Descriptor{ReadOnly: true, Description: "Build commit SHA"},
	)

	// Maintain local copies for hot paths
	var fps int64 = 15
	var frameID string = "camera"
	var exposure float64 = 0.02

	// Initial sync
	if p, ok := mgr.Get("camera.fps"); ok {
		fps = p.Value.Int64
	}
	if p, ok := mgr.Get("camera.frame_id"); ok {
		frameID = p.Value.Str
	}
	if p, ok := mgr.Get("camera.exposure"); ok {
		exposure = p.Value.Double
	}

	log.Printf("Node FQN: %s (name=%s)", n.FullyQualifiedName(), n.Name())
	// A tiny ticker to show we’re alive and using the latest params

	// Optional: hook /use_sim_time
	mgr.EnableUseSimTimeHook(func(enabled bool) {
		log.Printf("[param_demo] use_sim_time -> %v", enabled)
	})

	// Ticker that adapts to camera.fps
	tickerCh := make(chan struct{}, 1)
	rebuild := func() {
		select {
		case tickerCh <- struct{}{}:
		default:
		} // signal rebuild
	}
	mgr.OnSet(func(changes []params.Parameter) params.SetResult {
		for _, ch := range changes {
			switch ch.Name {
			case "camera.fps":
				fps = ch.Value.Int64
				log.Printf("camera.fps -> %d", fps)
				rebuild()
			case "camera.frame_id":
				frameID = ch.Value.Str
				log.Printf("camera.frame_id -> %s", frameID)
			case "camera.exposure":
				exposure = ch.Value.Double
				log.Printf("camera.exposure -> %.6f", exposure)
			}
		}
		return params.SetResult{Successful: true}
	})
	go func() {
		var t *time.Ticker
		start := func() {
			if t != nil {
				t.Stop()
			}
			period := time.Second
			if fps > 0 {
				period = time.Second / time.Duration(fps)
			}
			t = time.NewTicker(period)
		}
		start()
		for {
			select {
			case <-t.C:
				log.Printf("tick fps=%d frame_id=%s exposure=%.6f", fps, frameID, exposure)
			case <-tickerCh:
				start()
			}
		}
	}()

	// Spin until SIGINT/SIGTERM
	ctx, cancel := signal.NotifyContext(context.Background(), syscall.SIGINT, syscall.SIGTERM)
	defer cancel()
	if err := n.Spin(ctx); err != nil && ctx.Err() == nil {
		log.Printf("Spin error: %v", err)
	}
	_ = os.Stderr // keep imports honest
}
