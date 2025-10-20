package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

// tiny helpers
func i64(v int64) *int64     { return &v }
func f64(v float64) *float64 { return &v }

// very small arg parser that tolerates --ros-args etc.
func parseParamsFile(args []string) (string, []string) {
	out := make([]string, 0, len(args))
	var paramsPath string
	for i := 0; i < len(args); i++ {
		a := args[i]
		if a == "--params-file" && i+1 < len(args) {
			paramsPath = args[i+1]
			i++
			continue
		}
		out = append(out, a)
	}
	return paramsPath, out
}

func main() {
	// Parse ROS arguments (including parameter overrides like -p param:=value)
	rclArgs, restArgs, err := rclgo.ParseArgs(os.Args[1:])
	if err != nil {
		log.Fatalf("ParseArgs: %v", err)
	}

	// Init global context
	if err := rclgo.Init(rclArgs); err != nil {
		log.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Parse an optional --params-file from remaining args
	paramsPath, _ := parseParamsFile(restArgs)

	n, err := rclgo.NewNode("param_demo", "")
	if err != nil {
		log.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr, err := params.NewManager(n)
	if err != nil {
		log.Fatalf("params.NewManager: %v", err)
	}

	// If a YAML is provided, preload it first. Any names loaded here
	// will be considered "declared", and we won't redeclare them below.
	if paramsPath != "" {
		if err := params.LoadYAML(mgr, "param_demo", paramsPath); err != nil {
			log.Printf("[param_demo] LoadYAML(%s): %v", filepath.Base(paramsPath), err)
		} else {
			log.Printf("[param_demo] Loaded YAML params from %s", paramsPath)
		}
	}

	// Helper: declare a parameter only if it doesn't already exist.
	declareIfMissing := func(name string, v params.Value, d params.Descriptor) {
		if _, ok := mgr.Get(name); ok {
			return
		}
		if _, err := mgr.Declare(name, v, d); err != nil {
			log.Printf("[param_demo] declare %s: %v", name, err)
		}
	}

	// Declare our standard parameters (skip if YAML already provided them)
	declareIfMissing("camera.fps",
		params.Value{Kind: params.KindInt64, Int64: 15},
		params.Descriptor{MinInt: i64(1), MaxInt: i64(120), Description: "Output frame rate"},
	)
	declareIfMissing("camera.frame_id",
		params.Value{Kind: params.KindString, Str: "camera"},
		params.Descriptor{Description: "TF frame for published images"},
	)
	declareIfMissing("camera.exposure",
		params.Value{Kind: params.KindDouble, Double: 0.02},
		params.Descriptor{MinDouble: f64(0.001), MaxDouble: f64(0.1), Description: "Exposure time (s)"},
	)
	declareIfMissing("build.git_sha",
		params.Value{Kind: params.KindString, Str: "dev"},
		params.Descriptor{ReadOnly: true, Description: "Build commit SHA"},
	)

	// Apply CLI parameter overrides (e.g., --ros-args -p camera.fps:=30)
	if err := params.ApplyOverrides(mgr, n.Name(), rclArgs); err != nil {
		log.Printf("[param_demo] ApplyOverrides: %v", err)
	} else {
		log.Printf("[param_demo] Applied CLI parameter overrides")
	}

	// Maintain local copies for hot paths
	var fps int64 = 15
	var frameID string = "camera"
	var exposure float64 = 0.02

	// Initial sync (pull values loaded by YAML or default declarations)
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
		}
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
