local function load_config(package)
    return function()
        require('plugins.' .. package)
    end
end

-- Uncomment the line below to load only the nexessary plugins
-- local plugins2 = require('plugins.otherPlugins').plugins2
-- local plugins2 = {}
local plugins = {
    ---- ======================== first priority  =======================
    -- Telescope
    --
    {
        'nvim-telescope/telescope.nvim',
        branch = '0.1.x',
        dependencies = {
            'nvim-lua/plenary.nvim',
            {
                'nvim-telescope/telescope-fzf-native.nvim',
                build = 'make',
            },
            'molecule-man/telescope-menufacture', --?? needs exploration
            "nvim-telescope/telescope-live-grep-args.nvim",
        },
        config = load_config('PluginList.telescope'),
        cmd = 'Telescope',
    },
    {
        'stevearc/oil.nvim',
        opts = {},
        -- optional dependencies
        dependencies = { "nvim-tree/nvim-web-devicons" },
        config = load_config('PluginList.oil'),
        event = { 'vimenter' },
    },
    {
        "folke/tokyonight.nvim",
        dependencies = {
            "shatur/neovim-ayu",
            -- 'navarasu/onedark.nvim',
            -- "ellisonleao/gruvbox.nvim", --7/10
            -- "lunacookies/vim-colors-xcode",
            -- "catppuccin/nvim",
            -- "Mofiqul/dracula.nvim",

            -- "rebelot/kanagawa.nvim", --5/10
            -- 'nyoom-engineering/oxocarbon.nvim',
            -- "bluz71/vim-moonfly-colors"


        },
        config = load_config('PluginList.colorscheme'),
        lazy = false,
        priority = 1000,
    },

}

for _, plugin in ipairs(plugins2) do
    table.insert(plugins, plugin)
end

local lsp_servers = {
    'lua_ls',
    'clangd',
    'tsserver',
    'html-lsp',
}

local ts_parsers = {
    'javascript',
    'lua',
    "c",
}

return {
    plugins = plugins,
    -- lsp_servers = lsp_servers,
    -- ts_parsers = ts_parsers,
}
